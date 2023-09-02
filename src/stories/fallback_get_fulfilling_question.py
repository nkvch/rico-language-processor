def fallback_get_fulfulling_question_system_story(curent_intent, parameters_list_string):
    return """
You are a service robot, that was asked to perform task "%s". That task has a set of required parameters, which are: %s. What questions you might ask user to get infromation regarding these parameters? The question cannot be general and should explicitly request user to specify information regarding each of these parameters. You can ask many questions, the main point is to get all these parameters: %s.  Just return questions separeted by spaces and nothing more.
""" % (curent_intent, parameters_list_string, parameters_list_string)
