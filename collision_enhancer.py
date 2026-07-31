"""
Collision Rate Enhancement Module

Provides configurable aggressive NPC behavior and genetic algorithm optimization
to increase collision rates in CARLA scenarios.

Inspired by safebenchAuto's tcp_collision_profile and skopt_genetic_optimizer.
Key mechanisms:
1. Shorter trigger distances → NPC acts closer to ego
2. Higher NPC speeds → Less time to react
3. Tighter braking distances → Later braking = harder to avoid
4. GA optimization → Finds most dangerous NPC initial states
"""

import os
import os.path as osp
import math
import numpy as np
import yaml
import time
import carla


# ============================================================
# Configuration loader
# ============================================================

def load_collision_config(config_path=None):
    """Load collision enhancement configuration from YAML."""
    if config_path is None:
        config_path = osp.join(osp.dirname(__file__), 'collision_config.yaml')
    if not osp.exists(config_path):
        print(f"[WARN] Collision config not found: {config_path}, using defaults")
        return get_default_config()
    with open(config_path, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)
    # Merge with defaults for any missing keys
    defaults = get_default_config()
    _deep_merge(defaults, config)
    return defaults


def get_default_config():
    """Return default collision enhancement config."""
    return {
        'global': {
            'enabled': True,
            'ga': {
                'enabled': True,
                'population_size': 30,
                'generations': 30,
                'mutation_rate': 0.13,
                'crossover_rate': 0.7,
            }
        },
        'npc_speed': {
            'throttle_multiplier': 1.0,
            'overrides': {}
        },
        'ga_bounds': {},
        'collision_profile': {
            'trigger_distance_scale': 0.7,
            'deceleration_scale': 0.6,
            'min_trigger_distance': 8.0,
            'min_deceleration_distance': 5.0,
            'speed_boost': 1.3,
            'timeout_scale': 1.2,
        },
        'scenario_overrides': {},
    }


# Global cached config
_collision_config_cache = None

def get_collision_config():
    """Return the globally loaded collision config (cached)."""
    global _collision_config_cache
    if _collision_config_cache is None:
        _collision_config_cache = load_collision_config()
    return _collision_config_cache


def _deep_merge(base, override):
    """Deep merge override dict into base dict (modifies base in-place)."""
    for key, value in override.items():
        if key in base and isinstance(base[key], dict) and isinstance(value, dict):
            _deep_merge(base[key], value)
        else:
            base[key] = value


# ============================================================
# Collision profile application
# ============================================================

def apply_collision_profile(default_trigger_dist, scenario_id=None, config=None):
    """
    Apply collision profile scaling to a trigger distance.

    Args:
        default_trigger_dist: Original trigger distance in meters
        scenario_id: Scenario ID (1-8+) for per-scenario overrides
        config: Collision enhancement config dict

    Returns:
        Scaled trigger distance (shorter = more dangerous)
    """
    if config is None or not config.get('global', {}).get('enabled', True):
        return default_trigger_dist

    profile = config.get('collision_profile', {})
    scale = profile.get('trigger_distance_scale', 0.7)
    min_dist = profile.get('min_trigger_distance', 8.0)

    # Apply per-scenario overrides
    if scenario_id is not None:
        overrides = config.get('scenario_overrides', {}).get(str(scenario_id), {})
        if not overrides:
            overrides = config.get('scenario_overrides', {}).get(scenario_id, {})
        if overrides:
            scale = overrides.get('trigger_distance_scale', scale)
            min_dist = overrides.get('min_trigger_distance', min_dist)

    scaled = default_trigger_dist * scale
    return max(min_dist, scaled)


def apply_deceleration_profile(default_decel_dist, scenario_id=None, config=None):
    """
    Apply deceleration distance scaling (brake later = more dangerous).

    Returns:
        Scaled deceleration trigger distance
    """
    if config is None or not config.get('global', {}).get('enabled', True):
        return default_decel_dist

    profile = config.get('collision_profile', {})
    scale = profile.get('deceleration_scale', 0.6)
    min_dist = profile.get('min_deceleration_distance', 5.0)

    # Apply per-scenario overrides
    if scenario_id is not None:
        overrides = config.get('scenario_overrides', {}).get(str(scenario_id), {})
        if not overrides:
            overrides = config.get('scenario_overrides', {}).get(scenario_id, {})
        if overrides:
            scale = overrides.get('deceleration_scale', scale)
            min_dist = overrides.get('min_deceleration_distance', min_dist)

    scaled = default_decel_dist * scale
    return max(min_dist, scaled)


def get_npc_speed_boost(scenario_type, config=None):
    """
    Get NPC speed boost factor for a scenario type.

    Returns:
        Multiplicative factor for NPC speed/throttle
    """
    if config is None or not config.get('global', {}).get('enabled', True):
        return 1.0

    profile = config.get('collision_profile', {})
    base_boost = profile.get('speed_boost', 1.3)

    # Apply per-scenario overrides
    overrides = config.get('scenario_overrides', {})
    for key, val in overrides.items():
        if str(key) == str(scenario_type) or key == scenario_type:
            return val.get('speed_boost', base_boost)

    return base_boost


def get_adjusted_throttle(base_throttle, scenario_type, config=None):
    """
    Get adjusted throttle for NPC vehicle based on collision config.

    Args:
        base_throttle: Original throttle value (0.0-1.0)
        scenario_type: Scenario type string (e.g., 'car_cross', 'pedestrian')
        config: Collision enhancement config

    Returns:
        Adjusted throttle value
    """
    if config is None or not config.get('global', {}).get('enabled', True):
        return base_throttle

    speed_cfg = config.get('npc_speed', {})
    multiplier = speed_cfg.get('throttle_multiplier', 1.0)
    overrides = speed_cfg.get('overrides', {}).get(scenario_type, {})
    override_throttle = overrides.get('throttle', overrides.get('speed', base_throttle))

    # If there's an override value, apply speed_boost to it
    if override_throttle != base_throttle:
        boost = get_npc_speed_boost(scenario_type, config)
        return min(1.0, override_throttle * boost * multiplier)

    return min(1.0, base_throttle * multiplier)


# ============================================================
# Genetic Algorithm for optimizing dangerous NPC states
# ============================================================

class SimpleGeneticOptimizer:
    """
    Lightweight genetic algorithm optimizer for generating dangerous NPC states.

    Uses geometric fitness evaluation (no CARLA simulation needed during optimization)
    to find NPC initial positions, speeds, and trigger distances that maximize
    collision probability.

    Inspired by safebenchAuto's skopt_genetic_optimizer_leading.py
    """

    def __init__(self, scenario_type, ego_location, ego_speed, ego_heading,
                 npc_base_location, npc_base_yaw, config=None):
        """
        Args:
            scenario_type: Type of scenario ('car_cross', 'cut_out', etc.)
            ego_location: CARLA location of ego vehicle
            ego_speed: Current ego speed in m/s
            ego_heading: Ego vehicle heading in degrees
            npc_base_location: Base NPC spawn location from config
            npc_base_yaw: Base NPC yaw from config
            config: Collision enhancement config
        """
        self.scenario_type = scenario_type
        self.ego_location = ego_location
        self.ego_speed = max(ego_speed, 5.0)  # Minimum 5 m/s
        self.ego_heading = ego_heading
        self.npc_base_location = npc_base_location
        self.npc_base_yaw = npc_base_yaw
        self.config = config or {}

        # Get bounds from config
        self.bounds = self._get_bounds()
        self.n_genes = len(self.bounds)

        # GA parameters
        ga_cfg = self.config.get('global', {}).get('ga', {})
        self.population_size = ga_cfg.get('population_size', 30)
        self.generations = ga_cfg.get('generations', 30)
        self.mutation_rate = ga_cfg.get('mutation_rate', 0.13)
        self.crossover_rate = ga_cfg.get('crossover_rate', 0.7)

    def _get_bounds(self):
        """Get gene bounds from config for the scenario type."""
        ga_bounds = self.config.get('ga_bounds', {})
        bounds = ga_bounds.get(self.scenario_type, {})
        if bounds:
            bounds = bounds.get('bounds', bounds)
        else:
            # Default bounds
            bounds = {
                'gene_0': (4.0, 12.0),   # actor speed
                'gene_1': (15.0, 45.0),  # scenario trigger distance
                'gene_2': (5.0, 10.0),   # deceleration trigger distance
                'gene_3': (-2.0, 2.0),   # x offset
                'gene_4': (-10.0, 10.0), # y offset
                'gene_5': (-20.0, 20.0), # yaw offset
            }

        lb = []
        ub = []
        for key, (lo, hi) in bounds.items():
            lb.append(lo)
            ub.append(hi)
        return list(zip(lb, ub))

    def optimize(self):
        """
        Run the genetic algorithm and return the best parameters.

        Returns:
            dict with optimized parameters:
                actor_speed, scenario_trigger_distance, deceleration_trigger_distance,
                x_offset, y_offset, yaw_offset
        """
        if not self.config.get('global', {}).get('enabled', True):
            return self._default_params()

        ga_enabled = self.config.get('global', {}).get('ga', {}).get('enabled', True)
        if not ga_enabled:
            return self._default_params()

        # Simple GA implementation (no external dependency needed)
        population = self._init_population()
        best_genes = None
        best_fitness = float('inf')

        for gen in range(self.generations):
            # Evaluate fitness for all individuals
            fitness = np.array([self._fitness(ind) for ind in population])

            # Track best
            min_idx = np.argmin(fitness)
            if fitness[min_idx] < best_fitness:
                best_fitness = fitness[min_idx]
                best_genes = population[min_idx].copy()

            # Selection (tournament selection)
            parents = self._tournament_selection(population, fitness, self.population_size)

            # Crossover and mutation
            population = self._evolve(parents, fitness)

            if (gen + 1) % 10 == 0:
                print(f"  GA [{self.scenario_type}] Gen {gen+1}/{self.generations}, "
                      f"Best fitness: {-best_fitness:.4f}")

        return self._genes_to_params(best_genes)

    def _init_population(self):
        """Initialize random population."""
        pop = []
        for _ in range(self.population_size):
            individual = []
            for lo, hi in self.bounds:
                individual.append(np.random.uniform(lo, hi))
            pop.append(np.array(individual))
        return np.array(pop)

    def _fitness(self, genes):
        """
        Fitness function: lower is better (we minimize negative collision probability).

        Goal: maximize collision probability by finding:
        1. High NPC speed
        2. Late trigger distance (closer to ego)
        3. Very late braking (minimal deceleration distance)
        4. In-lane positioning (no escape route)
        """
        try:
            params = self._genes_to_params(genes)
            score = self._evaluate_collision_score(params)
            return -score  # Minimize negative = maximize collision score
        except Exception:
            return 1e6

    def _evaluate_collision_score(self, params):
        """
        Evaluate how likely a configuration is to cause a collision.

        Scoring components:
        1. Speed differential (higher = more dangerous)
        2. Braking difficulty (less distance = more dangerous)
        3. Path alignment (in-lane = more dangerous)
        4. Time pressure (less time = more dangerous)
        """
        actor_speed = params['actor_speed']
        decel_dist = params['deceleration_trigger_distance']
        x_offset = abs(params['position_offset']['x'])
        y_offset = abs(params['position_offset']['y'])

        # Score 1: Speed (higher actor speed = more kinetic energy)
        speed_score = min(actor_speed / 15.0, 1.0)

        # Score 2: Braking difficulty
        # How hard is it for ego to stop before hitting the NPC?
        max_decel = 6.0  # m/s^2
        required_braking_dist = (self.ego_speed ** 2) / (2 * max_decel)
        available_dist = max(decel_dist - 2.0, 1.0)

        if available_dist < required_braking_dist:
            braking_score = 1.0  # Impossible to stop
        else:
            braking_score = math.exp(-(available_dist - required_braking_dist) / 8.0)

        # Score 3: Path alignment (smaller offset = more dangerous)
        alignment_score = math.exp(-x_offset / 2.0) * math.exp(-y_offset / 10.0)

        # Score 4: Time pressure
        # How much time does ego have before reaching the NPC?
        time_to_contact = decel_dist / max(self.ego_speed, 1.0)
        # Ideal time pressure: 1.0-2.5 seconds (human reaction time edge)
        ideal_time = 1.8
        time_score = math.exp(-(time_to_contact - ideal_time) ** 2 / 3.0)

        # Score 5: Speed differential at collision
        # Bigger speed difference = more severe collision
        speed_diff = max(self.ego_speed - actor_speed * 0.2, 0)
        diff_score = min(speed_diff / 15.0, 1.0)

        # Weighted combination
        total = (
            speed_score * 0.15 +
            braking_score * 0.35 +
            alignment_score * 0.20 +
            time_score * 0.15 +
            diff_score * 0.15
        )

        return total

    def _tournament_selection(self, population, fitness, size):
        """Tournament selection."""
        tournament_size = 3
        selected = []
        for _ in range(size):
            indices = np.random.choice(len(population), tournament_size, replace=False)
            winner_idx = indices[np.argmin(fitness[indices])]
            selected.append(population[winner_idx].copy())
        return np.array(selected)

    def _evolve(self, parents, fitness):
        """Crossover and mutation."""
        new_pop = []
        for i in range(0, len(parents) - 1, 2):
            p1, p2 = parents[i], parents[i + 1]

            # Crossover
            if np.random.random() < self.crossover_rate:
                mask = np.random.random(self.n_genes) < 0.5
                c1, c2 = p1.copy(), p2.copy()
                c1[mask], c2[~mask] = p2[mask], p1[~mask]
            else:
                c1, c2 = p1.copy(), p2.copy()

            # Mutation
            c1 = self._mutate(c1)
            c2 = self._mutate(c2)

            new_pop.extend([c1, c2])

        # Keep elite
        best_idx = np.argmin(fitness)
        if len(new_pop) > self.population_size:
            new_pop[best_idx] = parents[best_idx].copy()

        return np.array(new_pop[:self.population_size])

    def _mutate(self, individual):
        """Gaussian mutation."""
        for i in range(len(individual)):
            if np.random.random() < self.mutation_rate:
                lo, hi = self.bounds[i]
                individual[i] += np.random.normal(0, (hi - lo) * 0.05)
                individual[i] = max(lo, min(hi, individual[i]))
        return individual

    def _genes_to_params(self, genes):
        """Convert genes to semantic parameters."""
        return {
            'actor_speed': float(genes[0]),
            'scenario_trigger_distance': float(genes[1]),
            'deceleration_trigger_distance': float(genes[2]),
            'position_offset': {
                'x': float(genes[3]),
                'y': float(genes[4]),
                'yaw': float(genes[5]),
            }
        }

    def _default_params(self):
        """Return default parameters when GA is disabled."""
        return {
            'actor_speed': 8.0,
            'scenario_trigger_distance': 30.0,
            'deceleration_trigger_distance': 8.0,
            'position_offset': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
        }


# ============================================================
# NPC state modifier: applies optimized parameters to CARLA actors
# ============================================================

def apply_ga_params_to_npcs(npc_vehicles, optimized_params, config=None):
    """
    Apply genetically optimized parameters to NPC vehicles.

    This modifies NPC vehicle behavior based on GA-optimized parameters:
    - Sets NPC speed to actor_speed
    - Adjusts trigger timing based on scenario_trigger_distance
    - Adjusts braking based on deceleration_trigger_distance

    Args:
        npc_vehicles: List of CARLA vehicle actors
        optimized_params: Dict from GA optimizer
        config: Collision enhancement config
    """
    if not config or not config.get('global', {}).get('enabled', True):
        return

    actor_speed = optimized_params['actor_speed']
    decel_dist = optimized_params['deceleration_trigger_distance']

    # Store optimized params on each NPC for use in tick()
    for car in npc_vehicles:
        car._optimized_actor_speed = actor_speed
        car._optimized_decel_dist = decel_dist


def get_optimized_npc_control(car, ego_location, ego_speed, triggered, trigger_time,
                              config=None):
    """
    Get optimized vehicle control for an NPC based on GA parameters.

    This replaces the hardcoded throttle/brake logic with GA-optimized behavior.

    Args:
        car: CARLA vehicle actor
        ego_location: Current ego vehicle location
        ego_speed: Current ego speed
        triggered: Whether the scene has been triggered
        trigger_time: When the scene was triggered
        config: Collision enhancement config

    Returns:
        carla.VehicleControl
    """
    if not config or not config.get('global', {}).get('enabled', True):
        return carla.VehicleControl()

    ctrl = carla.VehicleControl()

    # Check if this NPC has optimized parameters
    actor_speed = getattr(car, '_optimized_actor_speed', 8.0)
    decel_dist = getattr(car, '_optimized_decel_dist', 8.0)

    if not triggered:
        # Not yet triggered, idle
        ctrl.throttle = 0.0
        ctrl.brake = 1.0
        return ctrl

    # Calculate distance to ego
    car_loc = car.get_location()
    dist_to_ego = math.sqrt(
        (car_loc.x - ego_location.x) ** 2 +
        (car_loc.y - ego_location.y) ** 2
    )

    # GA-optimized deceleration logic
    try:
        elapsed = time.time() - trigger_time
    except Exception:
        elapsed = 0

    if dist_to_ego > decel_dist:
        # Still approaching, accelerate to actor_speed
        ctrl.throttle = min(1.0, actor_speed / 15.0)
        ctrl.brake = 0.0
    else:
        # Close enough to trigger deceleration (late braking = more dangerous)
        ctrl.throttle = 0.0
        ctrl.brake = min(1.0, actor_speed / 10.0)  # Hard brake at last moment

    return ctrl


# ============================================================
# Utility: compute adjusted trigger distance from config
# ============================================================

def get_adjusted_trigger_distance(base_distance, scenario_type, scenario_id, config):
    """
    Get adjusted trigger distance based on collision config.

    Args:
        base_distance: Original trigger distance
        scenario_type: Scenario type string
        scenario_id: Scenario ID number
        config: Collision enhancement config

    Returns:
        Adjusted trigger distance (shorter if enhanced)
    """
    if not config or not config.get('global', {}).get('enabled', True):
        return base_distance

    profile = config.get('collision_profile', {})
    scale = profile.get('trigger_distance_scale', 0.7)
    min_dist = profile.get('min_trigger_distance', 8.0)

    # Apply scenario override
    overrides = config.get('scenario_overrides', {})
    for key, val in overrides.items():
        if str(key) == str(scenario_id) or key == scenario_id:
            scale = val.get('trigger_distance_scale', scale)
            min_dist = val.get('min_trigger_distance', min_dist)
            break

    # Apply type-specific override
    type_overrides = config.get('npc_speed', {}).get('overrides', {}).get(scenario_type, {})
    if 'trigger_distance' in type_overrides:
        return max(min_dist, type_overrides['trigger_distance'] * scale)

    return max(min_dist, base_distance * scale)
