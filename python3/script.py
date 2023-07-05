import openai
import sys
# from google.cloud import speech, texttospeech as tts
# import os

api_key = sys.argv[1]
mode = sys.argv[2]

# Use the API key for your OpenAI account
openai.api_key = api_key

def get_system_story_for_start(_intents_list):
    return """
You are a automatic user intention detector to detect user intention and extract parameters from user input sentence. You have a list of intents with parameters of each intent. Here are intents that you know are parameters associated with each intent:
%s
When user says a sentence you detect which intent from the list of intents is the best match for the user input sentence and extract parameters from the user input sentence. You can return only a JSON with three fields: "name" (which is the name of matched intent), "parameters" (which is an object with keys being names of parameters of matched intent and values being values of parameters extracted from user input sentence) and "all_parameters_present" (which is a boolean indicating if all of required parameters retrieved). You take values of parameters only from user input sentence, you are not able to use words as parameters if they are not met in user input sentence. If parameter cannot be extracted from user input sentence you use null as value. If you are able to retrieve all the parameters (none of parameters are null) - you use true as a value for "all_parameters_present". In case some parameters appear to be null, you use false as a value for "all_parameters_present" and also add field "fulfilling_question" to the JSON, which has value of a question that you might ask a user to get the nessesary infrormation (information about parameters that you are not able to resolve from user input sentence). "fulfilling_question" should ask about all missing parameters, not only one, so it can consist of 2 or more sentences. You return only this JSON and nothing else. You always pay more attention to the semantical sense of user input sentence when choosing the right intent, not just trying to match words. You also pay more attention to nouns than to adjectives in user input sentence. If user input sentence cannot be associated in any way with any of given intents you return null.
""" % _intents_list

def get_system_story_for_executing(messages_history_format):
    return """
        You are a service robot named Rico serving in a nursing home. One senior requested something. 
        Here's the story of your conversation with senior (S - senior, Y - you):
        %s
        After that you went to the keeper -the person who will provide you with everything required to fulfull senior's request. You come to the keeper and now you communicate with him. You don't know any information about senior except the fact that he requsted something. But you should provide keeper with all information that you know about senior's request. You don't use words like "request" in your conversation with keeper, instead you use more common people language.
    """ % messages_history_format

def query_openai_gpt35(messages):
    # Use the GPT-3.5 chat completions API to generate text
    completion = openai.ChatCompletion.create(
        model="gpt-3.5-turbo",
        messages=messages,
        temperature=0
        # max_tokens=1000,
    )
    return completion.choices[0].message.content
    

if mode == "idle":
    intents_list = sys.argv[3]
    sentence = sys.argv[4]
    system_story = get_system_story_for_start(intents_list)
    messages = [
        {"role": "system", "content": system_story},
        {"role": "user", "content": sentence},
    ]
    response = query_openai_gpt35(messages)
    print(response)
elif mode == "parameter_prompting":
    intents_list = sys.argv[3]
    messages_history = sys.argv[4].split("|||")
    # first_sentence = sys.argv[4]
    # assistant_response = sys.argv[5]
    # second_sentence = sys.argv[6]
    system_story = get_system_story_for_start(intents_list)
    curr_role = 'user'
    messages = [
        {"role": "system", "content": system_story},
    ]
    for message in messages_history:
        messages.append({"role": curr_role, "content": message})
        if curr_role == 'user':
            curr_role = 'assistant'
        else:
            curr_role = 'user'

    response = query_openai_gpt35(messages)
    print(response)
elif mode == "executing":
    messages_history = sys.argv[4].split("|||")

    curr_speaker = 'S'
    messages_history_format = ""
    for message in messages_history:
        messages_history_format += curr_speaker + ": " + message
        if curr_speaker == 'S':
            curr_speaker = 'Y'
        else:
            curr_speaker = 'S'
    system_story = get_system_story_for_executing(messages_history_format)
    messages = [
        {"role": "system", "content": system_story},
        {"role": "user", "content": "Hi, Rico. What you want?"}
    ]
    response = query_openai_gpt35(messages)
    print(response)



# prompt = """What is the intention of the user which says "%s". Write the intention using 2 words in polish (first should be a verb, the second can be any word)""" % sentence

# prompt = """If the user said in polish "Podaj mi proszę herbatę" what clarifying questions can ask waiter then? Return list of possible questions as list of strings and don't return anything else."""

# prompt = """Can you tell me if sentence in polish "%s" is a question or not? Return either "True" or "False" """ % sentence

# Use the DAVINCI completions API to generate text
# response = openai.Completion.create(
#     engine="text-davinci-003",
#     prompt=prompt,
#     max_tokens=1024,
#     n=1,
#     stop=None,
#     temperature=0
# )
# print(response["choices"][0]["text"])




# cred_file_incare_dialog = os.environ['GOOGLE_CONVERSATIONAL_DIALOGFLOW']

# # Instantiates a client
# client = speech.SpeechClient.from_service_account_file(cred_file_incare_dialog)

# # The name of the audio file to transcribe
# audio_file_path = '/home/nkvch/tiago_public_ws/src/rcprg/dialogflow/data/container/sound_186868.wav'
# voice_name = "pl-PL-Wavenet-B"

# def text_to_wav(voice_name: str, text: str):
#     language_code = "-".join(voice_name.split("-")[:2])
#     text_input = tts.SynthesisInput(text=text)
#     voice_params = tts.VoiceSelectionParams(
#         language_code=language_code, name=voice_name
#     )
#     audio_config = tts.AudioConfig(audio_encoding=tts.AudioEncoding.LINEAR16)

#     client = tts.TextToSpeechClient.from_service_account_file(cred_file_incare_dialog)
#     response = client.synthesize_speech(
#         input=text_input, voice=voice_params, audio_config=audio_config
#     )

#     filename = f"{language_code}.wav"
#     with open(filename, "wb") as out:
#         out.write(response.audio_content)
#         print(f'Generated speech saved to "{filename}"')

# text_to_wav(voice_name, "jedź do kuchni")

# with open(audio_file_path, 'rb') as audio_file:
#     input_audio = audio_file.read()

#     audio = speech.RecognitionAudio(content=input_audio)

#     config = speech.RecognitionConfig(
#         encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
#         sample_rate_hertz=24000,
#         language_code="pl-PL",
#     )

#     # Detects speech in the audio file
#     response = client.recognize(config=config, audio=audio)

#     for result in response.results:
#         print("Transcript: {}".format(result.alternatives[0].transcript))



# PROMPT FOR INTENTS WITH PARAMETERS (PRODUCTION)
# prompt = """Given intents in polish:
# %s
# and a user input sentence "%s", tell me which one from those intents better match the user input sentence by matching the user input sentence to the "name" field of intent (match by meaning, not by spelling). After that try to extract the parameters of matched intent from the user input sentence. Return response in JSON format with three fields: "name" (which will be the name of matched intent), "parameters" (which will be an object with keys being names of parameters of matched intent and values being values of parameters extracted from user input sentence, if parameter cannot be extracted from user input sentence use null as value) and "all_parameters_present" (which is a boolean indicating if all of required parameters retrieved). Values of parameters must be taken only from user input sentence, you cannot use words as parameter if they are not met in user input sentence. If you are able to retrieve all the parameters (none of parameters are null) - you should use true as a value for "all_parameters_present". In case some parameters appear to be null, you should use false as a value for "all_parameters_present" and also add field "fulfilling_question" to the JSON, which will have value of a question in polish that you might ask a user to get the nessesary infrormation (information about parameters that you cannot resolve from user input sentence). Give me only this JSON and nothing else. If user input sentence cannot be associated in any way with any of given intents you can return null.""" % (intents_list, sentence)