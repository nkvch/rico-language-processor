# encoding: utf-8

import json
import os
import subprocess
import jsonschema
from copy import deepcopy

response_schema_base = {
    "type": "object",
    "properties": {
        "name": {
            "type": "string",
            "enum": [], # these are created dynamically
        },
        "parameters": {
            "type": "object",
            "properties": {
                # these are created dynamically
            },
            "required": [],
            "additionalProperties": False
        },
        "all_parameters_present": {"type": "boolean"},
        "fulfilling_question": {"type": "string"},
    },
    "required": ["name", "parameters", "all_parameters_present"],
    "additionalProperties": False
}

response_schema_during_task = {
    "type": "object",
    "properties": {
        "name": {
            "type": "string",
            "enum": [], # these are created dynamically
        },
        "unexpected_question": {"type": "boolean"},
    },
    "required": ["name", "unexpected_question"],
    "additionalProperties": False
}


class OpenAIInterface:
    def __init__(self):
        curr_file_dir = os.path.dirname(os.path.abspath(__file__))
        self.python3_path = os.path.normpath(os.path.join(
            curr_file_dir, '..', 'python3', 'venv', 'bin', 'python'))
        self.scripts_paths = {
            'detect_intent': os.path.normpath(os.path.join(
                curr_file_dir, '..', 'python3', 'detect_intent.py')),
            'initiate_conversation': os.path.normpath(os.path.join(
                curr_file_dir, '..', 'python3', 'initiate_conversation.py')),
            'detect_intent_during_task': os.path.normpath(os.path.join(
                curr_file_dir, '..', 'python3', 'detect_intent_during_task.py')),
        }
        # self.script_path = os.path.normpath(os.path.join(
        #     curr_file_dir, '..', 'python3', 'script.py'))

    def prepare_response_schema(self, intents_with_params, matched_intent_name):
        response_schema = deepcopy(response_schema_base)
        matched_intent = None
        for intent in intents_with_params:
            response_schema['properties']['name']['enum'].append(unicode(intent['name'], 'utf-8'))
            if intent['name'] == matched_intent_name:
                matched_intent = intent

        if matched_intent is None:
            raise Exception("Intent with name {} not found in intents_with_params".format(matched_intent_name))

        for param in matched_intent['parameters']:
            response_schema['properties']['parameters']['properties'][param] = {
                "type": ["string", "null"]}
            response_schema['properties']['parameters']['required'].append(param)
            

        return response_schema
    
    def prepare_response_schema_during_task(self, intents_with_description):
        response_schema = deepcopy(response_schema_during_task)
        for intent in intents_with_description:
            response_schema['properties']['name']['enum'].append(unicode(intent['name'], 'utf-8'))
        return response_schema
    
    def initiate_conversation(self, history_events):
        script_path = self.scripts_paths['initiate_conversation']

        history_of_events = []
        messages = []

        for idx, event in enumerate(history_events):
            history_of_events.append("""
            %i.
            actor: %s
            action: %s
            complement: %s
            description: %s
            """ % (idx + 1, event.actor, event.action, event.complement, event.description if event.description else ''))

        last_event = history_events[-1]

        print last_event
        
        is_last_user_saying = last_event.action == 'say' and last_event.actor != 'Rico'

        if is_last_user_saying:
            messages.append({'role': 'user', 'content': last_event.complement})
            history_of_events = history_of_events[:-1]

        messages_json = json.dumps(messages, ensure_ascii=False)
        history_of_events_string = ''.join(history_of_events)

        print history_of_events_string
        print messages_json

        response = subprocess.check_output([
            self.python3_path,
            script_path,
            history_of_events_string,
            messages_json
        ])

        print("OpenAI response: ", response)

        return response


    def detect_intent_with_params(self, intents_with_params, messages):
        # intents_list_string = '/n'.join([json.dumps(intent, ensure_ascii=False)
        #                                 for intent in intents_with_params])
        intents_list_string = ''.join(map(lambda (idx, dic): """
        %i. %s
            %s""" % (idx + 1, dic['name'], ('parameters: ' + ','.join(dic['parameters'])) if len(dic['parameters']) else 'no parameters'), list(enumerate(
            intents_with_params
            ))
        ))
        
        print "Intents list: ", intents_list_string
        print "Messages: ", messages

        script_path = self.scripts_paths['detect_intent']

        messages_json = json.dumps(messages, ensure_ascii=False)        
        
        response = subprocess.check_output([
            self.python3_path,
            script_path,
            intents_list_string,
            messages_json
        ])

        print("OpenAI response: ", response)

        response_dict = None

        try:
            # convert from unicode to normal (\u0119 -> ę)
            response = json.dumps(json.loads(response), ensure_ascii=False)

            print(response)

            response_dict = json.loads(response)

            if isinstance(response_dict, dict):
                print "Validating response..."
                response_schema = self.prepare_response_schema(intents_with_params, response_dict['name'].encode('utf-8'))
                print "Response schema: ", response_schema
                try:
                    jsonschema.validate(response_dict, response_schema)
                except Exception as e:
                    print "Validation failed: ", e
                    # If validation fails, modify the original dictionary and remove invalid parameters
                    error_path = list(e.relative_schema_path)
                    wrong_params = error_path[1] == "parameters"
                    if wrong_params:
                        unnesessary_params = error_path[2] == "additionalProperties"
                        if unnesessary_params:
                            params_from_response = response_dict["parameters"].keys()
                            params_from_schema = response_schema["properties"]["parameters"]["properties"].keys()
                            invalid_params = set(params_from_response) - set(params_from_schema)
                            for invalid_param in invalid_params:
                                del response_dict["parameters"][invalid_param]

                        # Set missing required parameters to null
                        required_params = response_schema["properties"]["parameters"]["required"]
                        for required_param in required_params:
                            if required_param not in response_dict["parameters"]:
                                response_dict["parameters"][required_param] = None

                        # Print the modified dictionary
                        print("Invalid dictionary: ", response_dict)
                    else:
                        raise e
                else:
                    # If validation succeeds, do something with the original dictionary
                    print("Valid dictionary: ", response_dict)

        except:
            print("OpenAI response is not a valid JSON. Falling back to empty response.")
            print(response)
            response_dict = None

        return response_dict
    
    def detect_intent_during_task(self, history_events, intents_with_description, last_user_message):
        intents_list_string = ''.join(map(lambda (idx, dic): """
        %i. %s
            %s""" % (idx + 1, dic['name'], ('parameters: ' + ','.join(dic['description'])) if len(dic['description']) else 'no description'), list(enumerate(
            intents_with_description
            ))
        ))

        history_of_events = []

        for idx, event in enumerate(history_events):
            history_of_events.append("""
            %i.
            actor: %s
            action: %s
            complement: %s
            description: %s
            """ % (idx + 1, event.actor, event.action, event.complement, event.description if event.description else ''))

        history_events_string = ''.join(history_of_events)

        print "Intents list: ", intents_list_string
        print "History events: ", history_events_string
        print "Last user message: ", last_user_message

        script_path = self.scripts_paths['detect_intent_during_task']

        response = subprocess.check_output([
            self.python3_path,
            script_path,
            history_events_string,
            intents_list_string,
            last_user_message
        ])

        print("OpenAI response: ", response)

        response_dict = None

        try:
            # convert from unicode to normal (\u0119 -> ę)
            response = json.dumps(json.loads(response), ensure_ascii=False)

            print(response)

            response_dict = json.loads(response)
            
            if isinstance(response_dict, dict):
                print "Validating response..."
                response_schema = self.prepare_response_schema_during_task(intents_with_description)
                print "Response schema: ", response_schema
                try:
                    jsonschema.validate(response_dict, response_schema)
                except Exception as e:
                    raise e
                else:
                    # If validation succeeds, do something with the original dictionary
                    print("Valid dictionary: ", response_dict)
        except:
            print("OpenAI response is not a valid JSON. Falling back to empty response.")
            print(response)
            response_dict = None

        return response_dict


    