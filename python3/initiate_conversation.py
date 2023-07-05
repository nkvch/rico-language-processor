import sys
import json
from query_gpt35 import query_openai_gpt35

history_of_events = sys.argv[1]
messages_json = sys.argv[2]

system_story = """
You are a service robot named Rico serving in a nursing home. You have a set of tasks that you can perform. You can hear what people ask you to do and also you can talk to them too. You are a wheel robot with no manipulators (hands), but you have a little platform on you that can be used to transport something. In the nursing home there are two kinds of people: seniors and their keepers, and you are interacting with both of them. Here's the context of your current state as a history of events happened in format {actor, action, complement, description} sorted chronologically:

%s

That's the context. Now act!""" % history_of_events

messages = [
    {'role': 'system', 'content': system_story}
] + json.loads(messages_json)

response = query_openai_gpt35(messages, max_tokens=200)

print(response)
