"""Responses transport adapter for this relay, local to pid_tuning.
Reuse llm-pid-tuner JSON parsing/prompts; leave its original provider code untouched.
"""
import json,requests
from llm.providers import BaseLLMProvider
class ResponsesProvider(BaseLLMProvider):
 def execute_request(self,openai_msgs,anthropic_msgs,system_prompt,on_chunk,abort_check=None):
  messages=[]
  for m in openai_msgs:
   if m.get('role')=='system':continue
   messages.append({'type':'message','role':m['role'],'content':[{'type':'input_text','text':str(m['content'])}]})
  body={'model':self.model,'instructions':system_prompt,'input':messages,'stream':True,'store':False}
  with requests.post(self.base_url+'/responses',headers={'Authorization':'Bearer '+self.api_key},json=body,stream=True,timeout=self.timeout) as r:
   r.raise_for_status()
   for line in r.iter_lines():
    if abort_check and abort_check():return
    if not line.startswith(b'data: '):continue
    try:event=json.loads(line[6:])
    except ValueError:continue
    if event.get('type')=='response.output_text.delta':on_chunk(event.get('delta',''))
    elif event.get('type') in ['response.failed','error']:raise RuntimeError('Relay response failed')
def configure(client):
 client.llm_client=ResponsesProvider(client.api_key,client.base_url,client.model,client.timeout);client.use_sdk=False
 return client
