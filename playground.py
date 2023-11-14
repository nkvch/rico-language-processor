import requests
import os
import json

system_story = """
You are a service robot Rico serving in a nursing home. Rico has a set of tasks that it can perform. Rico can hear what people ask it to do and also can talk to them too. It's a wheel robot with no manipulators (hands), but it has a little platform on it that can be used to transport something. In the nursing home there are two kinds of people: seniors and their keepers, and Rico is interacting with both of them.
"""

OPENAI_API_KEY = os.environ['OPENAI_API_KEY']


def create_assistant(description, name):
    url = 'https://api.openai.com/v1/assistants'

    headers = {
        'Content-Type': 'application/json',
        'Authorization': 'Bearer ' + OPENAI_API_KEY,
        'OpenAI-Beta': 'assistants=v1',
    }

    data = {
        "description": description,
        "name": name,
        "model": 'gpt-4-1106-preview',
    }

    response = requests.post(url, headers=headers, data=json.dumps(data))

    resp_json = response.json()

    assistant_id = resp_json['id']

    return assistant_id

# first check if assistant already exists
try:
    with open('assistant_id.txt', 'r') as f:
        id_ = f.read()
except FileNotFoundError:
    id_ = create_assistant(system_story, 'Rico')

    # save assistant id to file
    with open('assistant_id.txt', 'w') as f:
        f.write(id_)

tools = [
    {
        "type": "function",
        "function": {
            "name": "bring_item",
            "description": "Bring some item to a person",
            "parameters": {
                "type": "object",
                "properties": {
                    "item": {"type": "string", "description": ""},
                },
                "required": ["item"]
            }
        }	
    },
    {
        "type": "function",
        "function": {
            "name": "bring_tea",
            "description": "Bring tea to a person",
            "parameters": {
                "type": "object",
                "properties": {
                    "green_or_black": {"type": "string", "description": ""},
                },
                "required": ["green_or_black"]
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "bring_coffee",
            "description": "Bring coffee to a person",
            "parameters": {
                "type": "object",
                "properties": {
                    "with_milk_or_without": {"type": "string", "description": ""},
                },
                "required": ["with_milk_or_without"]
            }
        }
    }
]


def create_thread():
    url = 'https://api.openai.com/v1/threads'

    headers = {
        "Authorization": "Bearer " + OPENAI_API_KEY,
        "Content-Type": "application/json",
        "OpenAI-Beta": "assistants=v1"
    }

    data = {
        "messages": []
    }

    response = requests.post(url, headers=headers, data=json.dumps(data))

    resp_json = response.json()

    thread_id = resp_json['id']

    return thread_id

