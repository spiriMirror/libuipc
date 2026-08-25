'''CLI entry point for GPU benchmark and profiling tools.

Run, profile, and compare UIPC simulation benchmarks.

Examples::

    # Run a benchmark
    python -m uipc.cli.benchmark run --scene cube_ground --frames 10

    # Profile with Nsight Compute
    python -m uipc.cli.benchmark profile --scene cube_ground --frames 3

    # Compare two benchmark runs
    python -m uipc.cli.benchmark compare bench_before/ bench_after/

    # Create and enforce a same-runner performance baseline
    python -m uipc.cli.benchmark baseline bench_results/cube_ground -o baseline.json
    python -m uipc.cli.benchmark check bench_results/cube_ground --baseline baseline.json

    # List available scenes from uipc-assets
    python -m uipc.cli.benchmark list
'''
import sys
import argparse


def main():
    '''Benchmark and profile UIPC simulations with uipc.stats + Nsight Compute CLI'''
    parser = argparse.ArgumentParser(
        prog='uipc.cli.benchmark',
        description='Benchmark and profile UIPC GPU simulations',
    )
    sub = parser.add_subparsers(dest='command', help='Sub-command')

    # --- run ---
    p_run = sub.add_parser(
        'run', help='Run a headless benchmark and collect timer stats'
    )
    p_run.add_argument(
        '--scene', '-s', required=True, nargs='+',
        help='Asset name(s) from uipc-assets (e.g. cube_ground)',
    )
    p_run.add_argument(
        '--frames', '-n', type=int, default=10,
        help='Number of simulation frames (default: 10)',
    )
    p_run.add_argument(
        '--warmup', '-w', type=int, default=0,
        help='Warmup frames excluded from measurements (default: 0)',
    )
    p_run.add_argument(
        '--output', '-o', default='bench_results',
        help='Output directory (default: bench_results)',
    )

    # --- profile ---
    p_prof = sub.add_parser(
        'profile', help='Profile a scene with Nsight Compute CLI (ncu)'
    )
    p_prof.add_argument(
        '--scene', '-s', required=True,
        help='Asset name to profile',
    )
    p_prof.add_argument(
        '--frames', '-n', type=int, default=3,
        help='Number of frames to profile (default: 3)',
    )
    p_prof.add_argument(
        '--output', '-o', default='ncu_results',
        help='Output directory (default: ncu_results)',
    )
    p_prof.add_argument(
        '--kernel', '-k', default=None,
        help='Regex filter for kernel names (ncu -k)',
    )
    p_prof.add_argument(
        '--ncu-set', default='default',
        help='Nsight Compute section set (default: default; use full for duration metrics, requires admin)',
    )
    p_prof.add_argument(
        '--skip', type=int, default=0,
        help='Skip first N kernel launches (ncu -s)',
    )
    p_prof.add_argument(
        '--count', type=int, default=None,
        help='Max kernel launches to profile (ncu -c)',
    )
    p_prof.add_argument(
        '--ncu-path', default=None,
        help='Explicit path to ncu executable',
    )

    # --- analyze ---
    p_analyze = sub.add_parser(
        'analyze',
        help='Analyze bottlenecks from a benchmark result directory',
    )
    p_analyze.add_argument(
        'result_dir',
        help='Path to benchmark result directory (with timer_frames.json)',
    )
    p_analyze.add_argument(
        '--ncu-csv', default=None,
        help='Path to ncu CSV file to include kernel-level analysis',
    )
    p_analyze.add_argument(
        '--top', type=int, default=10,
        help='Number of top entries to show (default: 10)',
    )

    # --- compare ---
    p_cmp = sub.add_parser(
        'compare', help='Compare two benchmark result directories'
    )
    p_cmp.add_argument('before', help='Baseline benchmark result directory')
    p_cmp.add_argument('after', help='Optimized benchmark result directory')
    p_cmp.add_argument(
        '--output', '-o', default=None,
        help='Output directory for the comparison report',
    )

    # --- baseline ---
    p_baseline = sub.add_parser(
        'baseline', help='Create a versioned performance baseline'
    )
    p_baseline.add_argument(
        'result_dirs', nargs='+',
        help='Benchmark result directories to include',
    )
    p_baseline.add_argument(
        '--output', '-o', required=True,
        help='Baseline JSON output path',
    )
    p_baseline.add_argument(
        '--max-regression-percent', type=float, default=10.0,
        help='Allowed regression for enforced metrics (default: 10)',
    )

    # --- check ---
    p_check = sub.add_parser(
        'check', help='Check benchmark results against a baseline'
    )
    p_check.add_argument(
        'result_dirs', nargs='+',
        help='Benchmark result directories to check',
    )
    p_check.add_argument(
        '--baseline', required=True,
        help='Baseline JSON path',
    )
    p_check.add_argument(
        '--max-regression-percent', type=float, default=None,
        help='Override every baseline regression allowance',
    )
    p_check.add_argument(
        '--allow-environment-mismatch', action='store_true',
        help='Report but do not fail on runner/build compatibility differences',
    )
    p_check.add_argument(
        '--json-output', default=None,
        help='Optional path for the structured check report',
    )

    # --- list ---
    sub.add_parser(
        'list', help='List available scenes from uipc-assets'
    )

    args = parser.parse_args()

    if args.command is None:
        parser.print_help()
        sys.exit(0)

    if args.command == 'run':
        _cmd_run(args)
    elif args.command == 'profile':
        _cmd_profile(args)
    elif args.command == 'analyze':
        _cmd_analyze(args)
    elif args.command == 'compare':
        _cmd_compare(args)
    elif args.command == 'baseline':
        _cmd_baseline(args)
    elif args.command == 'check':
        _cmd_check(args)
    elif args.command == 'list':
        _cmd_list()
    else:
        parser.print_help()
        sys.exit(1)


def _cmd_run(args):
    from uipc import Scene
    from uipc.assets import load as load_asset
    from uipc.profile import session

    if args.frames <= 0:
        raise ValueError('--frames must be greater than zero')
    if args.warmup < 0:
        raise ValueError('--warmup must be non-negative')

    print(
        f'Running benchmark: scenes={args.scene}, frames={args.frames}, '
        f'warmup={args.warmup}'
    )
    print(f'Output: {args.output}')
    print()

    results = []
    for name in args.scene:
        scene = Scene(Scene.default_config())
        load_asset(name, scene)
        with session(scene, name=name, output_dir=args.output) as benchmark:
            if args.warmup:
                benchmark.advance(args.warmup)
            benchmark.profile(args.frames)
        r = benchmark.result
        results.append(r)

    print()
    print('--- Summary ---')
    for r in results:
        if r.get('error'):
            print(f'  FAIL  {r["name"]}: {r["error"][:80]}')
        else:
            print(f'  OK    {r.get("summary", r["name"])}')


def _cmd_profile(args):
    from uipc import Scene
    from uipc.assets import load as load_asset
    from uipc.profile import nsight

    print(f'Profiling: scene={args.scene}, frames={args.frames}')

    scene = Scene(Scene.default_config())
    load_asset(args.scene, scene)

    result = nsight.run(
        scene,
        num_frames=args.frames,
        output_dir=args.output,
        name=args.scene,
        ncu_path=args.ncu_path,
        kernel_filter=args.kernel,
        ncu_set=args.ncu_set,
        skip_kernels=args.skip,
        max_kernels=args.count,
    )
    print()
    if result['ncu_rep']:
        print(f'Report:      {result["ncu_rep"]}')
        print(f'CSV:         {result["csv_path"]}')
        if result.get('report_md'):
            print(f'Analysis:    {result["report_md"]}')
        if result.get('report_json'):
            print(f'JSON report: {result["report_json"]}')
        print(f'Kernels profiled: {len(result["rows"])}')
        print()
        # Print the saved analysis (already generated by profile)
        if result.get('report_md'):
            import pathlib
            md = pathlib.Path(result['report_md']).read_text(encoding='utf-8')
            print(md)
    else:
        print('No .ncu-rep file produced.  Check ncu output:')
        print(result.get('stderr', ''))


def _cmd_analyze(args):
    from uipc.profile import load_result
    from uipc.profile.nsight import parse_ncu_csv, analyze_bottlenecks

    data = load_result(args.result_dir)
    stats = data.get('stats')

    ncu_rows = None
    if args.ncu_csv:
        ncu_rows = parse_ncu_csv(args.ncu_csv)

    md = analyze_bottlenecks(stats=stats, ncu_rows=ncu_rows, top_n=args.top)
    print(md)


def _cmd_compare(args):
    from uipc.profile import compare

    md = compare(args.before, args.after, output_dir=args.output)
    print(md)


def _cmd_baseline(args):
    from uipc.profile import create_baseline

    payload = create_baseline(
        args.result_dirs,
        args.output,
        max_regression_percent=args.max_regression_percent,
    )
    print(
        f"Wrote {args.output} with "
        f"{len(payload['benchmarks'])} benchmark(s)."
    )


def _cmd_check(args):
    import json
    import pathlib

    from uipc.profile import check_baseline
    from uipc.profile.baseline import format_check_report

    report = check_baseline(
        args.result_dirs,
        args.baseline,
        max_regression_percent=args.max_regression_percent,
        require_environment_match=not args.allow_environment_mismatch,
    )
    print(format_check_report(report))
    if args.json_output:
        path = pathlib.Path(args.json_output)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    if not report['passed']:
        raise SystemExit(1)


def _cmd_list():
    try:
        from uipc.assets import list_assets
        names = list_assets()
        print(f'Available scenes ({len(names)}):')
        for name in names:
            print(f'  - {name}')
    except ImportError:
        print('uipc.assets not available.  Install huggingface_hub:')
        print('  pip install huggingface_hub')
    except Exception as e:
        print(f'Error listing assets: {e}')


if __name__ == '__main__':
    main()
