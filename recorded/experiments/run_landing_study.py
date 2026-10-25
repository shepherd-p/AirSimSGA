"""Real ProjectAirSim only. Each trial preserves images, telemetry and failures."""
import argparse
import asyncio
import json
from datetime import datetime
from pathlib import Path
from ag.storage import ROOT, write_json
from ag.landing_runtime import run_trial


async def main(args):
    config=json.loads(args.config.read_text(encoding='utf-8'))
    if args.timeout is not None:config['timeout_sec']=args.timeout
    if args.controller:config['controller']=args.controller
    speeds=args.speeds if args.speeds is not None else config['speeds_mps']
    repeats=args.repeats if args.repeats is not None else config['repeats']
    if repeats<1 or not speeds or any(s<0 for s in speeds):
        raise ValueError('Provide nonnegative speeds and at least one repeat')
    output=args.output or ROOT/'outputs/landing'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    output.mkdir(parents=True,exist_ok=False)
    write_json(output/'suite_config.json',dict(config,speeds_mps=speeds,repeats=repeats,probe=args.probe))
    results=[]
    for speed in speeds:
        for repeat in range(1,repeats+1):
            name=f'trial_{len(results)+1:03d}_v{speed:.2f}_{config["controller"]}'
            print(f'OUTPUT {output/name}',flush=True)
            result=await run_trial(config,output/name,speed,repeat,args.probe)
            results.append(dict(result,directory=name,speed_mps=speed,repeat=repeat))
            write_json(output/'summary.json',{'trials':results,'successes':sum(bool(r['success']) for r in results)})
            if result['status']=='error':
                print('Stopped suite on runtime error; earlier evidence retained.',flush=True)
                return 2
    print(f'OUTPUT {output}',flush=True)
    return 0 if all(r['success'] or r['status']=='probe_completed' for r in results) else 1


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=ROOT/'configs/landing/study.json')
    p.add_argument('--speeds',type=float,nargs='+')
    p.add_argument('--repeats',type=int)
    p.add_argument('--timeout',type=float)
    p.add_argument('--controller',choices=['predictive','baseline'])
    p.add_argument('--probe',action='store_true',help='Validate real vision and geometry without descending to contact')
    p.add_argument('--output',type=Path)
    args=p.parse_args()
    raise SystemExit(asyncio.run(main(args)))
