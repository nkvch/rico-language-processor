#!/usr/bin/env python
# encoding: utf8

import rospy
import tiago_msgs.msg
import os
import actionlib
from std_msgs.msg import String, Bool
from conversation_msgs.srv import GetScenariosIntentsWithParams
from language_processor.srv import DetectIntentAndRetrieveParams, DetectIntentAndRetrieveParamsResponse
from language_processor.srv import InitiateConvBasedOnCtx, InitiateConvBasedOnCtxResponse
from rico_context.srv import GetContext
from task_database.srv import GetScenarioInputs, GetScenarioInputsResponse
from openai_interface import OpenAIInterface
from copy import deepcopy
import json

# convert unicode to utf-8 (\u0119 -> \xc4\x99)
def encode_dict(d):
    return {key.encode('utf-8'): encode_dict(value) if isinstance(value, dict) else value.encode('utf-8') if isinstance(value, unicode) else value for key, value in d.items()}

# class ConversationState:
#     def __init__(self):
#         self.conversation_state = 'idle'
#         self.messages = []

#     def add_message(self, message):
#         self.messages.append(message)

#     def openai_response_callback(self, response):
#         matched = response is not None
#         all_parameters_present = response['all_parameters_present'] if matched else False
#         fulfilling_question = response['fulfilling_question'] if matched and not all_parameters_present else None

#         if fulfilling_question is not None:
#             self.conversation_state = 'parameter_prompting'
#             self.messages.append(fulfilling_question)
#         else:
#             self.messages.append('Okay!')
#             self.conversation_state = 'executing'


#     def is_conversation_idle(self):
#         return self.conversation_state == 'idle'

#     def is_parameter_prompting(self):
#         return self.conversation_state == 'parameter_prompting'


def main():
    rospy.init_node('language_processor', anonymous=True)
    get_scenarios_intents_with_params = rospy.ServiceProxy(
        'get_scenarios_intents_with_params', GetScenariosIntentsWithParams)
    get_context = rospy.ServiceProxy('/context/get', GetContext)
    get_scenario_inputs = rospy.ServiceProxy(
        'get_scenario_inputs', GetScenarioInputs)
    rospy.wait_for_service('get_scenarios_intents_with_params')
    rospy.wait_for_service('/context/get')

    # conversation_state = ConversationState()

    openai_interface = OpenAIInterface()

    def initiate_conv_based_on_ctx(req):
        rico_history_events = get_context().events

        filtered_events = []
        for event in rico_history_events:
            if event.actor != 'system':
                filtered_events.append(event)

        response = openai_interface.initiate_conversation(filtered_events)

        return InitiateConvBasedOnCtxResponse(response)

    def detect_intent_and_retrieve_params(req):
        scenarios_intents_with_params = get_scenarios_intents_with_params(
        ).scenarios_intents_with_params
        s_i_with_params_dict = dict(list(map(lambda siwp: (siwp.intent_name, {
            'scenario_id': siwp.scenario_id, 'name': siwp.intent_name, 'parameters': siwp.params}), scenarios_intents_with_params)))
        s_i_with_params_openai = deepcopy(s_i_with_params_dict.values())

        for s_i_with_params in s_i_with_params_openai:
            del s_i_with_params['scenario_id']
            for param in s_i_with_params['parameters']:
                if param.startswith('question_'):
                    s_i_with_params['parameters'].remove(param)
                    s_i_with_params['parameters'].append(
                        param.replace('question_', ''))

        rico_history_events = get_context().events

        is_in_task = False
        curr_scenario_id = None
        curr_task_name = None
        messages = []
        events = []

        for event in rico_history_events:
            if event.actor == 'system' and event.action == 'trigger scenario':
                is_in_task = True
                curr_scenario_id = int(event.complement)
            else:
                events.append(event)
            if event.actor == 'system' and event.action == 'finish scenario':
                is_in_task = False
                curr_scenario_id = None
            if event.action == 'start performing':
                is_in_task = True
                curr_task_name = event.complement
            if event.action == 'finish performing':
                is_in_task = False
                curr_task_name = None
            if event.action == 'say':
                messages.append({'role': 'assistant', 'content': event.complement})
            if event.action == 'hear':
                messages.append({'role': 'user', 'content': event.complement})

        if curr_scenario_id is not None and curr_task_name is not None:
            scenario_inputs = get_scenario_inputs(curr_scenario_id).inputs
            in_task_intents = []
            for input in scenario_inputs:
                in_task_intents.append({
                    'name': input.intent,
                    'description': input.description,
                })
            last_user_sentence = messages[-1]['content']
            
            response = openai_interface.detect_intent_during_task(
                events, in_task_intents, last_user_sentence)
        else:
            response = openai_interface.detect_intent_with_params(
                s_i_with_params_openai, messages)
            
        print response

        # conversation_state.openai_response_callback(response)

        matched = response is not None

        if matched:
            response = encode_dict(response)
            detected_scenario_intent = s_i_with_params_dict[response['name']]

        scenario_id = detected_scenario_intent['scenario_id'] if matched else -1
        intent_name = detected_scenario_intent['name'] if matched else ''
        all_parameters_present = response['all_parameters_present'] if matched else False
        retrieved_param_names = []
        retrieved_param_values = []
        unretrieved_param_names = []

        if matched:
            for key, value in response['parameters'].items():
                if value is not None:
                    retrieved_param_names.append(key)
                    retrieved_param_values.append(value)
                else:
                    unretrieved_param_names.append(key)
        
        fulfilling_question = response['fulfilling_question'] if matched and not all_parameters_present else ''

        return DetectIntentAndRetrieveParamsResponse(
            matched,
            scenario_id,
            intent_name,
            all_parameters_present,
            retrieved_param_names,
            retrieved_param_values,
            unretrieved_param_names,
            fulfilling_question,
        )
    
    rospy.Service('detect_intent_and_retrieve_params',
                  DetectIntentAndRetrieveParams, detect_intent_and_retrieve_params)
    
    rospy.Service('initiate_conv_based_on_ctx',
                  InitiateConvBasedOnCtx, initiate_conv_based_on_ctx)

    rospy.spin()


if __name__ == '__main__':
    main()