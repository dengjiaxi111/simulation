"""Path-aware tuning harness using llm-pid-tuner's client and guardrails.
The stock thermal convergence/rollback scoring is deliberately not used.
No API credentials are copied into this directory. Offline search is explicit.
"""
import argparse,json,os,sys,subprocess,time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parents[1]/'llm-pid-tuner'))
from gains import source_gains
from pid_safety import apply_pid_guardrails
from core.config import CONFIG
CONFIG['PID_MAX_INCREASE_RATIO']=1.5
W_LIMITS={'p':{'min':.01,'max':.28,'max_increase_ratio':1.5},'i':{'min':0,'max':.5,'max_increase_ratio':1.5},'d':{'min':0,'max':.001,'max_increase_ratio':1.5}}
S_LIMITS={'p':{'min':2,'max':30,'max_increase_ratio':1.5},'i':{'min':0,'max':1,'max_increase_ratio':1.5},'d':{'min':.01,'max':.8,'max_increase_ratio':1.5}}
def pid(g):return dict(zip(('p','i','d'),g))
def bounded(old,new):
    return [v for a,b,l in [(old[:3],new[:3],W_LIMITS),(old[3:],new[3:],S_LIMITS)] for v in apply_pid_guardrails(pid(a),pid(b),l)[0].values()]
def score(s):
    if not s['completed']:return 1e6
    return s['path_max']+s['velocity_vector_rmse']*.5+s['final_error']*.25

def main():
    p=argparse.ArgumentParser();p.add_argument('--initial-gains',type=float,nargs=6);p.add_argument('--rounds',type=int,default=6);p.add_argument('--shapes',nargs='+',default=['straight','S','U','circle']);p.add_argument('--size',type=float,default=16);p.add_argument('--speed',type=float,default=2);p.add_argument('--llm',action='store_true');p.add_argument('--repeats',type=int,default=3);a=p.parse_args()
    output=ROOT/'results'/time.strftime('tuning_%Y%m%d_%H%M%S');output.mkdir(parents=True);os.chdir(ROOT)
    best=a.initial_gains or source_gains();best_score=float('inf');history=[];consecutive=0;model=None
    if a.llm:
        from llm.client import LLMTuner
        from relay_provider import configure
        config=json.loads((ROOT/'.llm_config.json').read_text()) if (ROOT/'.llm_config.json').exists() else {}
        key=os.environ.get('LLM_API_KEY',config.get('api_key'));url=os.environ.get('LLM_API_BASE_URL',config.get('base_url'));name=os.environ.get('LLM_MODEL_NAME',config.get('model'))
        if not all([key,url,name]):raise RuntimeError('Set LLM_API_KEY, LLM_API_BASE_URL, LLM_MODEL_NAME; not configured')
        model=configure(LLMTuner(key,url,name,emit_console=False,timeout=55))
    candidate=best.copy()
    for i in range(a.rounds):
        summaries=[]
        for shape in a.shapes:
            f=output/f'{i:02d}_{shape}.json'
            subprocess.run([sys.executable,str(ROOT/'trial.py'),'--shape',shape,'--size',str(a.size),'--speed',str(a.speed),'--gains',*map(str,candidate),'--output',str(f)],check=True,timeout=150)
            summaries.append(json.loads(f.read_text())['summary'])
        value=max(score(s) for s in summaries);passed=all(s['passed'] for s in summaries)
        history.append(dict(round=i,gains=candidate,score=value,passed=passed,tests=summaries))
        if value<best_score:best=candidate.copy();best_score=value
        consecutive=consecutive+1 if passed and candidate==best else 0
        (output/'history.json').write_text(json.dumps(dict(best=best,best_score=best_score,consecutive_passes=consecutive,history=history),indent=2))
        if consecutive>=a.repeats:print('PROGRAM_VERIFIED',best);break
        if passed:candidate=best.copy();continue
        if model:
            result=model.analyze(json.dumps(history[-1]),json.dumps(history[:-1]),tuning_mode='generic',prompt_context={'controller_count':2,'controller_1':'wheel speed PID in SI','controller_2':'steer angle PID, derivative is measured angular velocity','objective':'All moving samples cross-track <= 0.1m; no terminal-only scoring. Maintain requested speed.','wheel_limits':W_LIMITS,'steer_limits':S_LIMITS,'pid_limits':W_LIMITS,'pid_limits_are_runtime_enforced':True,'secondary_pid_limits':S_LIMITS})
            if not result:raise RuntimeError('LLM returned no candidate; best recorded, no unverified fallback')
            c1=result.get('controller_1');c2=result.get('controller_2')
            if not c1 or not c2:raise RuntimeError('Expected both controller_1 and controller_2')
            candidate=bounded(best,[float(c[k]) for c in [c1,c2] for k in ('p','i','d')])
        else:
            # Deterministic baseline search. This is not an LLM call.
            options=[(3,1.25),(5,1.5),(0,1.25),(1,1.25),(3,.8),(5,.75)]
            k,factor=options[i%len(options)];candidate=best.copy();candidate[k]*=factor;candidate=bounded(best,candidate)
    print('BEST_TESTED',best,'score',best_score,'full certification',consecutive>=a.repeats)
if __name__=='__main__':main()
