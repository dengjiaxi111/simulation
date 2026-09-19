"""Ask the user's configured model using measured curves; never log credentials."""
import argparse,json,os,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT.parents[1]/'llm-pid-tuner'))
from llm.client import LLMTuner
from relay_provider import configure
from tune import bounded,W_LIMITS,S_LIMITS
p=argparse.ArgumentParser();p.add_argument('--files',nargs='+',required=True);p.add_argument('--current',type=float,nargs=6,required=True);p.add_argument('--output',required=True);a=p.parse_args();traces=[]
for name in a.files:
 d=json.loads(Path(name).read_text());rows=d['rows'];trace=[]
 for r in rows[::max(1,len(rows)//60)]:
  trace.append({k:r.get(k) for k in ['t','pose','reference','actual','desired','speed','angular_velocity','motor_telemetry','sim_time','velocity','cmd','command_chain','joints','tf_age']})
 traces.append(dict(summary=d['summary'],curve=trace))
c=json.loads((ROOT/'.llm_config.json').read_text());out=Path(a.output).resolve();os.chdir(ROOT)
m=LLMTuner(c['api_key'],c['base_url'],c['model'],emit_console=False,timeout=55)
configure(m)
context={'controller_count':2,'controller_1':'wheel speed PID','controller_2':'steer angle PID, D=-Kd*actual steer velocity','secondary_pid':dict(zip(('p','i','d'),a.current[3:])),'pid_limits':W_LIMITS,'secondary_pid_limits':S_LIMITS,'pid_limits_are_runtime_enforced':True,'objective':'All moving samples cross-track<=0.1m, final<=0.1m, velocity vector RMSE<=0.15m/s. Return both controller_1 and controller_2 p/i/d. Do not claim done without all tests. Same wheel gains applied to 4 wheels, same steer gains to 4 steer joints.','physics_dt':.001,'wheel_inertia_kgm2':[.00017243,.00017243,.0001487,.0001487],'wheel_torque_limit_Nm':2,'steer_torque_limit_Nm':5,'integral_clamp':1,'current_pid':dict(zip(('p','i','d'),a.current[:3])),'notes':'Existing controller code unchanged. No steering-error drive reduction. No yaw hold. The supplied summary stage identifies either feedforward plant or full_navigationros2_with_LIO. Full Nav2 tests use actual planner initial paths and real velocity conversion. Do not treat localization delays or obstacle-induced failure as PID-only issues. Only six gains may change; not force limits, controller code, navigation parameters, or physical properties.'}
result=m.analyze(json.dumps(traces),json.dumps({'current_gains':a.current}),tuning_mode='generic',prompt_context=context)
if not result:raise RuntimeError('No usable model result')
c1=result.get('controller_1');c2=result.get('controller_2')
if not c1 or not c2:raise RuntimeError('Model did not return both controllers')
g=bounded(a.current,[float(q[k]) for q in [c1,c2] for k in ('p','i','d')]);out.write_text(json.dumps(dict(model=c['model'],raw=result,gains=g),indent=2));print('GUARDED_CANDIDATE',g,flush=True)
