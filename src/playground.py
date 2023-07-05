#!/usr/bin/env python
# encoding: utf8

import os
from google.cloud import speech, texttospeech as tts
from openai_interface import OpenAIInterface

openai_key = os.environ['OPENAI_API_KEY']

openai_interface = OpenAIInterface(openai_key)

response = openai_interface.detect_intent_with_params(
    [{'name': 'jedź do', 'parameters': ['miejsce']},
     {'name': 'obróć się', 'parameters': []},
     {'name': 'podaj przedmiot', 'parameters': ['przedmiot']},
     {'name': 'podaj herbatę', 'parameters': ['przedmiot', 'czarna_czy_zielona']},
     {'name': 'podaj kawę', 'parameters': ['przedmiot', 'czarna_czy_biala']},
     {'name': 'odebrałem', 'parameters': []},
     {'name': 'potwierdzam', 'parameters': []},
     {'name': 'podałem', 'parameters': []},
     {'name': 'co wiezesz', 'parameters': []},
     {'name': 'obróć się', 'parameters': []}],
     "podaj kaszę"
)

print(response)

# cred_file_incare_dialog = os.environ['GOOGLE_CONVERSATIONAL_DIALOGFLOW']

# # Instantiates a client
# client = speech.SpeechClient.from_service_account_file(cred_file_incare_dialog)

# # The name of the audio file to transcribe
# audio_file_path = '/home/nkvch/tiago_public_ws/src/rcprg/dialogflow/data/container/sound_186868.wav'
# voice_name = "pl-PL-Wavenet-B"

# def text_to_wav(voice_name, text):
#     language_code = "-".join(voice_name.split("-")[:2])
#     text_input = tts.types.SynthesisInput(text=text)
#     voice_params = tts.types.VoiceSelectionParams(
#         language_code=language_code, name=voice_name
#     )
#     audio_config = tts.types.AudioConfig(audio_encoding=tts.enums.AudioEncoding.LINEAR16)

#     client = tts.TextToSpeechClient.from_service_account_file(cred_file_incare_dialog)
#     response = client.synthesize_speech(
#         text_input, voice_params, audio_config
#     )

#     filename = "output.wav"
#     with open(filename, "wb") as out:
#         out.write(response.audio_content)
#         print "Generated speech saved to {}".format(filename)

# text_to_wav(voice_name, "jedź do kuchni")
