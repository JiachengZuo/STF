"""Shared, bounded weather catalog used by scenario expanders.

Version 2 corrects legacy out-of-range values. Existing JSON is not rewritten.
Snow-named presets are visual approximations, not snowfall/ice physics.
"""

from __future__ import annotations

WEATHER_CATALOG_VERSION = 2

WEATHER_PRESET_ORDER = (
    "sunny", "cloudy", "overcast", "light_rain", "heavy_rain", "fog",
    "wind", "sandstorm", "night", "dusk", "dawn", "cold_fog",
    "rainy_dusk", "snowy_light", "snowy_heavy",
)


def build_weather_catalog():
    """Return the existing 15-preset, eight-level weather catalog."""
    groups = {name: [] for name in WEATHER_PRESET_ORDER}
    for i in range(8):
        groups["sunny"].append({
            "cloudiness": i * 5, "dust_storm": 0.0,
            "fog_density": 0.0, "fog_distance": 100.0, "fog_falloff": 1.0,
            "mie_scattering_scale": 0.03, "precipitation": 0.0,
            "precipitation_deposits": 0.0,
            "rayleigh_scattering_scale": 0.0331, "scattering_intensity": 1.0,
            "sun_altitude_angle": -30 + i * 15, "sun_azimuth_angle": 120.0,
            "wetness": 0.0, "wind_intensity": 5.0,
        })
        groups["cloudy"].append({
            "cloudiness": 30 + i * 8, "dust_storm": 0.0,
            "fog_density": 0.0, "fog_distance": 100.0, "fog_falloff": 1.0,
            "mie_scattering_scale": 0.03, "precipitation": 0.0,
            "precipitation_deposits": 0.0,
            "rayleigh_scattering_scale": 0.0331, "scattering_intensity": 0.95,
            "sun_altitude_angle": 45.0, "sun_azimuth_angle": 120.0,
            "wetness": 0.0, "wind_intensity": 10.0 + i * 3,
        })
        groups["overcast"].append({
            "cloudiness": 50 + i * 2.5, "dust_storm": 0.0,
            "fog_density": 0.0, "fog_distance": 100.0, "fog_falloff": 1.0,
            "mie_scattering_scale": 0.03, "precipitation": 0.0,
            "precipitation_deposits": 0.0,
            "rayleigh_scattering_scale": 0.0331,
            "scattering_intensity": 0.7 - i * 0.05,
            "sun_altitude_angle": 25.0, "sun_azimuth_angle": 120.0,
            "wetness": 0.0, "wind_intensity": 15.0 + i * 4,
        })
        groups["light_rain"].append({
            "cloudiness": 10.0, "dust_storm": 0.0,
            "fog_density": i * 4, "fog_distance": 80 - i * 5,
            "fog_falloff": 1.0, "mie_scattering_scale": 0.03,
            "precipitation": 5 + i * 4,
            "precipitation_deposits": 10 + i * 6,
            "rayleigh_scattering_scale": 0.0331,
            "scattering_intensity": 0.75 - i * 0.06,
            "sun_altitude_angle": 20.0, "sun_azimuth_angle": 120.0,
            "wetness": 20 + i * 8, "wind_intensity": 18 + i * 4,
        })
        groups["heavy_rain"].append({
            "cloudiness": 0.0, "dust_storm": 0.0,
            "fog_density": i * 8, "fog_distance": 60 - i * 6,
            "fog_falloff": 1.0, "mie_scattering_scale": 0.03,
            "precipitation": 50 + i * 7,
            "precipitation_deposits": 60 + i * (40 / 7),
            "rayleigh_scattering_scale": 0.0331,
            "scattering_intensity": 0.5 - i * 0.05,
            "sun_altitude_angle": 15.0, "sun_azimuth_angle": 120.0,
            "wetness": 70 + i * (30 / 7), "wind_intensity": 25 + i * 6,
        })
        groups["fog"].append({
            "cloudiness": 65 + i * 5, "dust_storm": 0.0,
            "fog_density": 30 + i * 10, "fog_distance": 70 - i * 8,
            "fog_falloff": 0.8 - i * 0.05, "mie_scattering_scale": 0.03,
            "precipitation": 0.0, "precipitation_deposits": 0.0,
            "rayleigh_scattering_scale": 0.0331,
            "scattering_intensity": 0.5 - i * 0.05,
            "sun_altitude_angle": 35.0, "sun_azimuth_angle": 120.0,
            "wetness": 0.0, "wind_intensity": 5.0 + i * 2,
        })
        groups["wind"].append({
            "cloudiness": 70 + i * 4, "dust_storm": 0.0,
            "fog_density": 0.0, "fog_distance": 100.0, "fog_falloff": 1.0,
            "mie_scattering_scale": 0.03, "precipitation": 0.0,
            "precipitation_deposits": 0.0,
            "rayleigh_scattering_scale": 0.0331, "scattering_intensity": 0.9,
            "sun_altitude_angle": 45.0, "sun_azimuth_angle": 120.0,
            "wetness": 0.0, "wind_intensity": 20 + i * 8,
        })
        groups["sandstorm"].append({
            "cloudiness": 0.0, "dust_storm": 44 + i * 8,
            "fog_density": 40 + i * (60 / 7), "fog_distance": 60 - i * 7,
            "fog_falloff": 0.6 - i * 0.05, "mie_scattering_scale": 0.03,
            "precipitation": 0.0, "precipitation_deposits": 0.0,
            "rayleigh_scattering_scale": 0.0331,
            "scattering_intensity": 0.4 - i * 0.04,
            "sun_altitude_angle": 45.0, "sun_azimuth_angle": 120.0,
            "wetness": 0.0, "wind_intensity": 30 + i * 10,
        })
        groups["night"].append({
            "cloudiness": 100.0, "dust_storm": 0.0,
            "fog_density": 0.0, "fog_distance": 100.0, "fog_falloff": 1.0,
            "mie_scattering_scale": 0.03, "precipitation": 0.0,
            "precipitation_deposits": 0.0,
            "rayleigh_scattering_scale": 0.0331,
            "scattering_intensity": 0.6 - i * 0.07,
            "sun_altitude_angle": -55.0 - i * 5, "sun_azimuth_angle": 120.0,
            "wetness": 0.0, "wind_intensity": 5.0,
        })
        groups["dusk"].append({
            "cloudiness": 40 + i * 6, "dust_storm": 0.0,
            "fog_density": 0.0, "fog_distance": 100.0, "fog_falloff": 1.0,
            "mie_scattering_scale": 0.03, "precipitation": 0.0,
            "precipitation_deposits": 0.0,
            "rayleigh_scattering_scale": 0.0331,
            "scattering_intensity": 0.85 - i * 0.06,
            "sun_altitude_angle": 15.0 - i * 4, "sun_azimuth_angle": 150.0,
            "wetness": 0.0, "wind_intensity": 8.0 + i * 2,
        })
        groups["dawn"].append({
            "cloudiness": 30 + i * 5, "dust_storm": 0.0,
            "fog_density": 5 + i * 3, "fog_distance": 90 - i * 6,
            "fog_falloff": 0.9, "mie_scattering_scale": 0.03,
            "precipitation": 0.0, "precipitation_deposits": 0.0,
            "rayleigh_scattering_scale": 0.0331,
            "scattering_intensity": 0.75 + i * 0.03,
            "sun_altitude_angle": -10.0 + i * 5, "sun_azimuth_angle": 60.0,
            "wetness": 10.0 + i * 5, "wind_intensity": 6.0 + i * 2,
        })
        groups["cold_fog"].append({
            "cloudiness": 50 + i * 5, "dust_storm": 0.0,
            "fog_density": 20 + i * (80 / 7), "fog_distance": 80 - i * 9,
            "fog_falloff": 0.75 - i * 0.04, "mie_scattering_scale": 0.03,
            "precipitation": 0.0,
            "precipitation_deposits": 5.0 + i * 3,
            "rayleigh_scattering_scale": 0.0331,
            "scattering_intensity": 0.6 - i * 0.05,
            "sun_altitude_angle": 20.0, "sun_azimuth_angle": 90.0,
            "wetness": 15.0 + i * 6, "wind_intensity": 4.0 + i * 1.5,
        })
        groups["rainy_dusk"].append({
            "cloudiness": 100.0, "dust_storm": 0.0,
            "fog_density": 10 + i * 6, "fog_distance": 75 - i * 7,
            "fog_falloff": 0.85, "mie_scattering_scale": 0.03,
            "precipitation": 15 + i * 5,
            "precipitation_deposits": 30 + i * 8,
            "rayleigh_scattering_scale": 0.0331,
            "scattering_intensity": 0.6 - i * 0.06,
            "sun_altitude_angle": 10.0 - i * 3, "sun_azimuth_angle": 140.0,
            "wetness": 40 + i * (60 / 7), "wind_intensity": 15 + i * 4,
        })
        groups["snowy_light"].append({
            "cloudiness": 80 + i * 2.5, "dust_storm": 0.0,
            "fog_density": 15 + i * 8, "fog_distance": 70 - i * 6,
            "fog_falloff": 0.8, "mie_scattering_scale": 0.05,
            "precipitation": 10 + i * 6,
            "precipitation_deposits": 20 + i * 10,
            "rayleigh_scattering_scale": 0.02,
            "scattering_intensity": 0.65 - i * 0.05,
            "sun_altitude_angle": 30.0, "sun_azimuth_angle": 120.0,
            "wetness": 30 + i * 8, "wind_intensity": 12 + i * 3,
        })
        groups["snowy_heavy"].append({
            "cloudiness": 100.0, "dust_storm": 0.0,
            "fog_density": 35 + i * (65 / 7), "fog_distance": 55 - i * 7,
            "fog_falloff": 0.7 - i * 0.04, "mie_scattering_scale": 0.07,
            "precipitation": 40 + i * 8,
            "precipitation_deposits": 50 + i * (50 / 7),
            "rayleigh_scattering_scale": 0.015,
            "scattering_intensity": 0.45 - i * 0.05,
            "sun_altitude_angle": 25.0, "sun_azimuth_angle": 120.0,
            "wetness": 60 + i * (40 / 7), "wind_intensity": 22 + i * 6,
        })
    return groups


def all_weather_profiles():
    """Return 120 named profiles suitable for scenario generation."""
    catalog = build_weather_catalog()
    return [
        {
            "id": "{}_{:02d}".format(preset, level),
            "preset": preset,
            "level": level,
            "parameters": dict(catalog[preset][level - 1]),
        }
        for preset in WEATHER_PRESET_ORDER
        for level in range(1, 9)
    ]
