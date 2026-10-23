"""Opt-in local runners. Bare --scenario keeps the upstream runner semantics."""
import importlib
import sys

RUNNERS = {
    '1a': 'speed_limit_1a_cli',
    '2a': 'stf_run_2a',
    '2d': 'cross_right_2d_cli',
    '3b_1': 'scripts.run_3b_1',
    '5a': 'scripts.run_5a',
    '5c': 'scripts.run_5c',
    '5d': 'emergency_brake_5d_cli',
    '5e': 'curve_emergency_5e_cli',
    '6c': 'station_stop_6c_cli',
}

# These runners retain their own argparse-based main() and read sys.argv.
ARGV_MAIN_RUNNERS = frozenset(('2a', '3b_1', '5a', '5c'))


def select_runner(argv, *, force_local=False):
    """Return (scenario or None, arguments); import no simulator/model modules."""
    argv = list(argv)
    modes, short_codes, scenario_codes, rest = [], [], [], []
    i = 0
    while i < len(argv):
        item = argv[i]
        if item == '--':
            rest.extend(argv[i:])
            break
        if item in ('--runner', '--scenario'):
            if i+1 >= len(argv):
                # Preserve the original parser's error unless local mode is requested.
                if item == '--runner' or force_local or short_codes or 'local' in modes:
                    raise ValueError(item+' requires a value')
                rest.append(item)
                i += 1
                continue
            value = argv[i+1]
            (modes if item == '--runner' else scenario_codes).append(value)
            if item == '--scenario':
                rest.extend((item, value))
            i += 2
            continue
        if item.startswith('--runner='):
            modes.append(item.split('=', 1)[1])
        elif item.startswith('--scenario='):
            scenario_codes.append(item.split('=', 1)[1])
            rest.append(item)
        elif item.startswith('-') and item.lstrip('-') in RUNNERS:
            short_codes.append(item.lstrip('-'))
        else:
            rest.append(item)
        i += 1
    if any(mode not in ('local', 'legacy') for mode in modes) or len(set(modes)) > 1:
        raise ValueError('--runner must select either local or legacy')
    if 'legacy' in modes and (force_local or short_codes):
        raise ValueError('--runner legacy conflicts with a local scenario selector')
    local = force_local or bool(short_codes) or 'local' in modes
    if not local:
        return None, rest
    codes = {code.lower().replace('.', '') for code in scenario_codes+short_codes}
    if len(codes) != 1:
        raise ValueError('select exactly one local scenario: '+', '.join(RUNNERS))
    code = next(iter(codes))
    if code not in RUNNERS:
        raise ValueError('no local runner for scenario '+code)
    forwarded = []
    i = 0
    while i < len(rest):
        if rest[i] == '--':
            forwarded.extend(rest[i:]); break
        if rest[i] == '--scenario':
            i += 2; continue
        if rest[i].startswith('--scenario='):
            i += 1; continue
        forwarded.append(rest[i]); i += 1
    return code, forwarded


def dispatch(argv=None, *, force_local=False):
    """None means continue the original main; an integer is a local exit status."""
    original = list(sys.argv[1:] if argv is None else argv)
    if force_local and original in (['--help'], ['-h']):
        print('usage: run_stf.py --scenario {'+','.join(RUNNERS)+'} [scenario options]')
        print('Local aliases: '+', '.join('-'+code for code in RUNNERS))
        print('Use --scenario CODE --help for scenario-specific options.')
        return 0
    try:
        code, remaining = select_runner(original, force_local=force_local)
    except ValueError as exc:
        print('STF runner: '+str(exc), file=sys.stderr)
        return 2
    if code is None:
        if argv is None and remaining != original:
            sys.argv[1:] = remaining
        return None
    module_name = RUNNERS[code]
    print('[STF local runner] {} -> {}'.format(code, module_name), flush=True)
    module = importlib.import_module(module_name)
    if code == '1a':
        result = module.main(remaining)
    elif code in ARGV_MAIN_RUNNERS:
        saved = sys.argv[:]
        try:
            sys.argv[1:] = ['--scenario', code]+remaining
            result = module.main()
        finally:
            sys.argv[:] = saved
    else:
        result = module.main('run', remaining)
    return 0 if result is None else result
