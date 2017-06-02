"""
GE.EVES Office Robot project
Program to determine if a name spoken to Alexa is present in a dynamoDB database
If there is a match then the dynamoDB entry is modified to signify to 
the GE.EVES Robot that it should proceed to the corresponding coordinates
"""

from __future__ import print_function

 # JSON encoder and decoder
import json

# Amazon Web Services SDK for Python
import boto3

# Attributes for dynamoDB queries
from boto3.dynamodb.conditions import Key, Attr
from botocore.exceptions import ClientError

# --------------- Global variables ---------------------------------------------

# Access keys necessary to query and modify dynamoDB
ACCESS_KEY='AKIAJE3KLYOD2MKHQFCA'
SECRET_KEY='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'

# Alexa Skill ID
alexa_skill_id = 'amzn1.ask.skill.bfcd7d3d-370e-475c-b926-4af489a68b14'

# Alexa skill and dyanmoDB variables
first_name_alexa = 'Name'
last_name_alexa = 'Name'
dynamodb = boto3.resource(
    'dynamodb',
	aws_access_key_id=ACCESS_KEY,
	aws_secret_access_key=SECRET_KEY,
	)
table_name = 'IDO_Studio'
table = dynamodb.Table(table_name)
index_name = 'First_Name-Last_Name-index'
office_id = 'office'
first_name_db = '1000'
last_name_db = '1000'


# --------------- Helpers that build all of the responses ----------------------

def build_speechlet_response(title, output, reprompt_text, should_end_session):
    return {
        'outputSpeech': {
            'type': 'PlainText',
            'text': output
        },
        'card': {
            'type': 'Simple',
            'title': title,
            'content': output
        },
        'reprompt': {
            'outputSpeech': {
                'type': 'PlainText',
                'text': reprompt_text
            }
        },
        'shouldEndSession': should_end_session
    }

def build_response(session_attributes, speechlet_response):
    return {
        'version': '1.0',
        'sessionAttributes': session_attributes,
        'response': speechlet_response
    }


# --------------- Functions that control the skill's behavior ------------------

def get_welcome_response():
    """ If we wanted to initialize the session to have some attributes we could
    add those here
    """

    session_attributes = {}
    card_title = "Welcome to the Industrial Design Studio!"
    speech_output = "Welcome to the industrial design studio. " \
                    "I'm Jeeves. I can help guide you to the office of anyone that works here. " \
                    "Just tell me the first and last name of the person you're seeking. "
    # If the user either does not reply to the welcome message or says something
    # that is not understood, they will be prompted again with this text.
    reprompt_text = "Sorry, I didn't catch that. " \
                    "Please say the first and last name of the person you need to find. "
    should_end_session = False
    return build_response(session_attributes, build_speechlet_response(
        card_title, speech_output, reprompt_text, should_end_session))


def handle_session_end_request():
    card_title = "GE.EVES:  Come back soon!"
    speech_output = "Thank you for visiting eye-dee-oh.  See you next time!"
    # Setting this to true ends the session and exits the skill.
    should_end_session = True
    return build_response({}, build_speechlet_response(
        card_title, speech_output, None, should_end_session))

def ask_for_confirmation(first_name_db, last_name_db, office_id):
    session_attributes = {}
    card_title = "Are you looking for " + first_name_db + " " + last_name_db + "'s office? "
    session_attributes = {"firstNameDB": first_name_db, "lastnameDB": last_name_db, "officeID": office_id, "confirmation": "confirm"}
 
    speech_output = "Just to confirm, are you looking for " + first_name_db + " " +  last_name_db + "'s office? "
    reprompt_text = "Sorry, I didn't hear your response. " \
            "Are you looking for " + first_name_db + last_name_db + "'s office? "\
	    "Say yes to confirm, or no if I have the wrong name."
    should_end_session = False

    return build_response(session_attributes, build_speechlet_response(
        card_title, speech_output, None, should_end_session))
        
def no_user_found():

    session_attributes = {}
    card_title = "Sorry, I couldn't find a match for that name."
    speech_output = "Sorry, I couldn't find a match for that name. " \
		    "Please repeat the name of the person you're seeking or say, stop, to cancel your search. "
    reprompt_text = "I'm not sure whose office you want to find. " \
            "Please repeat the name of the person you're seeking or say, stop, to cancel your search. "
    should_end_session = False

    return build_response(session_attributes, build_speechlet_response(
        card_title, speech_output, reprompt_text, should_end_session))

def multi_user_found():
    session_attributes = {}
    card_title = "I found multiple people with similar names."
    speech_output = "I found multiple people with a similar name. " \
            "Please repeat the name of the person you're seeking or say stop to cancel your search. "
    reprompt_text = "Please repeat the name of the person you're seeking or say stop to cancel your search. "
    should_end_session = False
    
    return build_response(session_attributes, build_speechlet_response(
        card_title, speech_output, reprompt_text, should_end_session))
        
def lets_go_response(intent, session):
    
    if session.get('attributes', {}) and "firstNameDB" in session.get('attributes', {}):
        first_name_db = session['attributes']['firstNameDB']
        card_title = "I found " + first_name_db + "'s office.  Follow me!"
        speech_output = "No problem. " \
                "I know just where to find " + first_name_db +"'s office. " \
                "But first, please insert one dollar.  Ha ha.  Just kidding. " \
                "I'll take you there.  Just follow me. "
        should_end_session = True
        
        return build_response({}, build_speechlet_response(
        card_title, speech_output, None, should_end_session))
    
    else:
        
        return error_response()

def error_response():
    
    session_attributes = {}
    card_title = "Problem encountered."
    speech_output = "My software has encountered a problem. " \
            "To restart your request, say computer, start jeeves bought. "
    reprompt_text = "Shutting down.  To restart your request, say computer, start jeeves bought. "
    should_end_session = True
    
    return build_response(session_attributes, build_speechlet_response(
        card_title, speech_output, reprompt_text, should_end_session))
        
def repeat_request():
    
    session_attributes = {}
    card_title = "Wrong user found"
    speech_output = "Ok.  Let's try again. " \
            "Please say the name of the person you're seeking or say stop to cancel your search. "
    reprompt_text = "Please say the name of the person you're seeking or say stop to cancel your search.  "
    should_end_session = False
    
    return build_response(session_attributes, build_speechlet_response(
        card_title, speech_output, reprompt_text, should_end_session))    


# --------------- Data manipulation with dynamoDB ---------------
def find_user(intent, session):
    session_attributes = {}

    # Get First Name from Intent and create first name variations for searching
    if "First_Name" in intent["slots"]:
        first_name_alexa = intent["slots"]["First_Name"]["value"]
        first_name_alexa_lc = first_name_alexa.lower()
        first_name_two_letters = first_name_alexa_lc[0:2]
        first_name_first_letter = first_name_alexa_lc[0:1]

    # Get Last Name from Intent and create last name variations for searching
    if "Last_Name" in intent["slots"]:
        last_name_alexa = intent["slots"]["Last_Name"]["value"]
        last_name_alexa_lc = last_name_alexa.lower()
        last_name_two_letters = last_name_alexa_lc[0:2]
        last_name_first_letter = last_name_alexa_lc[0:1]
 
    # Query the table to look for name matches starting with highest probability search and 
    # decreasing probability with subsequent searches

    if "First_Name" in intent["slots"] and "Last_Name" in intent["slots"]:
        dbresponse = table.query(
        TableName = table_name,
        IndexName = index_name,
        KeyConditionExpression=Key('First_Name').eq(first_name_alexa_lc) & Key('Last_Name').eq(last_name_alexa_lc),
        )
        
        if dbresponse['Count'] == 0:
            dbresponse = table.scan(
            TableName = table_name,
            IndexName = index_name,
            FilterExpression=Key('First_Name').eq(first_name_alexa_lc) & Attr('Last_Name').begins_with(last_name_two_letters),
            )
            
            if dbresponse['Count'] == 0:
                dbresponse = table.scan(
                TableName = table_name,
                IndexName = index_name,
                FilterExpression=Attr('First_Name').begins_with(first_name_two_letters) & Attr('Last_Name').begins_with(last_name_two_letters),
                )
                
                if dbresponse['Count'] == 0:
                    dbresponse = table.scan(
                        TableName = table_name,
                        IndexName = index_name,
                        FilterExpression=Attr('First_Name').begins_with(first_name_two_letters) & Attr('Last_Name').begins_with(last_name_first_letter),
                        )
                        
                    if dbresponse['Count'] == 0:
                        dbresponse = table.scan(
                        TableName = table_name,
                        IndexName = index_name,
                        FilterExpression=Attr('First_Name').begins_with(first_name_first_letter) & Attr('Last_Name').begins_with(last_name_two_letters),
                        )
                        
                        if dbresponse['Count'] == 0:
                            dbresponse = table.query(
                            TableName = table_name,
                            IndexName = index_name,
                            KeyConditionExpression=Key('First_Name').eq(first_name_alexa_lc),
                            )
                            
                            if dbresponse['Count'] == 0:
                                dbresponse = table.scan(
                                TableName = table_name,
                                IndexName = index_name,
                                FilterExpression=Attr('Last_Name').eq(last_name_alexa_lc),
                                )
   
                                if dbresponse['Count'] == 0:
                                    dbresponse = table.scan(
                                    TableName = table_name,
                                    IndexName = index_name,
                                    FilterExpression=Attr('First_Name').begins_with(first_name_first_letter) & Attr('Last_Name').begins_with(last_name_first_letter),
                                    )
                                    
                                    if dbresponse['Count'] == 0:
                                        
                                        return no_user_found()
                                        
        if dbresponse['Count'] == 1:
            
            # Find Office ID in database associated with the first name
            global office_id
            global first_name_db
            global last_name_db
            office_id = dbresponse[u'Items'][0][u'Office']
            first_name_db = dbresponse[u'Items'][0][u'First_Name']
            last_name_db = dbresponse[u'Items'][0][u'Last_Name']
            
            return ask_for_confirmation(first_name_db, last_name_db, office_id)
            
        else:
            
            return multi_user_found()
            
    elif "First_Name" in intent["slots"] and not "Last_Name" in intent["slots"]:
        dbresponse = table.query(
        TableName = table_name,
        IndexName = index_name,
        KeyConditionExpression=Key('First_Name').eq(first_name_alexa_lc),
        )
        
        if dbresponse['Count'] == 0:
            dbresponse = table.scan(
            TableName = table_name,
            IndexName = index_name,
            FilterExpression=Attr('First_Name').begins_with(first_name_two_letters),
            )
            
            if dbresponse['Count'] == 0:
                dbresponse = table.scan(
                TableName = table_name,
                IndexName = index_name,
                FilterExpression=Attr('First_Name').begins_with(first_name_first_letter),
                )
                
                if dbresponse['Count'] == 0:
                    return no_user_found()
                    
        if dbresponse['Count'] == 1:
            # Find Office ID in database associated with the first name
            office_id = dbresponse[u'Items'][0][u'Office']
            first_name_db = dbresponse[u'Items'][0][u'First_Name']
            last_name_db = dbresponse[u'Items'][0][u'Last_Name']
            
            return ask_for_confirmation(first_name_db, last_name_db, office_id)
            
        else:
                
                return multi_user_found()
                
    elif "Last_Name" in intent["slots"] and not "First_Name" in intent["slots"]:
        dbresponse = table.query(
        TableName = table_name,
        IndexName = index_name,
        KeyConditionExpression=Key('Last_Name').eq(last_name_alexa_lc),
        )
        
        if dbresponse['Count'] == 0:
            dbresponse = table.scan(
            TableName = table_name,
            IndexName = index_name,
            FilterExpression=Attr('Last_Name').begins_with(last_name_two_letters),
            )
            
            if dbresponse['Count'] == 0:
                dbresponse = table.scan(
                TableName = table_name,
                IndexName = index_name,
                FilterExpression=Attr('Last_Name').begins_with(last_name_first_letter),
                )
                
                if dbresponse['Count'] == 0:
                    
                    return no_user_found()
                    
        if dbresponse['Count'] == 1:
            # Find Office ID in database associated with the first name
            office_id = dbresponse[u'Items'][0][u'Office']
            first_name_db = dbresponse[u'Items'][0][u'First_Name']
            last_name_db = dbresponse[u'Items'][0][u'Last_Name']
            
            return ask_for_confirmation(first_name_db, last_name_db, office_id)
        
        else:
            
            return multi_user_found()
            
    else:
        
        return no_user_found()

def confirmed_user(intent, session):
    
    if session.get('attributes', {}) and "officeID" in session.get('attributes', {}):
        office_id = session['attributes']['officeID']
        
        # Define value of variable to modify
        locate_office = 'TRUE'
        
        # Update the Locate_Office entry to equal the value set for locate_office
        response = table.update_item(
            Key = {
            'Office' : office_id,
                
            },
            UpdateExpression = "set Locate_Office = :o",
            ExpressionAttributeValues = {
            ':o' : locate_office
            },
        )
        
        return lets_go_response(intent, session)
    
    else:
        
        return error_response()

# --------------- Events ------------------

def on_session_started(session_started_request, session):
    """ Called when the session starts """

    print("on_session_started requestId=" + session_started_request['requestId']
          + ", sessionId=" + session['sessionId'])


def on_launch(launch_request, session):
    """ Called when the user launches the skill without specifying what they
    want
    """

    print("on_launch requestId=" + launch_request['requestId'] +
          ", sessionId=" + session['sessionId'])
    # Dispatch to your skill's launch
    return get_welcome_response()


def on_intent(intent_request, session):
    """ Called when the user specifies an intent for this skill """

    print("on_intent requestId=" + intent_request['requestId'] +
          ", sessionId=" + session['sessionId'])

    intent = intent_request['intent']
    intent_name = intent_request['intent']['name']

    if session.get('attributes', {}) and "confirmation" in session.get('attributes', {}):
        if intent_name == "YesIntent":
            return confirmed_user(intent, session)
        elif intent_name == "NoIntent":
            return repeat_request()

    # Dispatch to your skill's intent handlers if not a Yes or No intent
    if intent_name == "FindUser":
        return find_user(intent, session)
    elif intent_name == "AMAZON.HelpIntent":
        return get_welcome_response()
    elif intent_name == "AMAZON.CancelIntent" or intent_name == "AMAZON.StopIntent":
        return handle_session_end_request()
    else:
        raise ValueError("Invalid intent")


def on_session_ended(session_ended_request, session):
    """ Called when the user ends the session.

    Is not called when the skill returns should_end_session=true
    """
    print("on_session_ended requestId=" + session_ended_request['requestId'] +
          ", sessionId=" + session['sessionId'])
    # add cleanup logic here
    # added the following as an exit message from the robot
    return handle_session_end_request()


# --------------- Main handler ------------------

def lambda_handler(event, context):
    """ Route the incoming request based on type (LaunchRequest, IntentRequest,
    etc.) The JSON body of the request is provided in the event parameter.
    """
    print("event.session.application.applicationId=" +
          event['session']['application']['applicationId'])

    if (event["session"]["application"]["applicationId"] !=
            alexa_skill_id):
        raise ValueError("Invalid Application ID")

    if event['session']['new']:
        on_session_started({'requestId': event['request']['requestId']},
                           event['session'])

# Determine the type of event coming from the Alexa skill
    if event['request']['type'] == "LaunchRequest":
        return on_launch(event['request'], event['session'])
    elif event['request']['type'] == "IntentRequest":
        return on_intent(event['request'], event['session'])
    elif event['request']['type'] == "SessionEndedRequest":
        return on_session_ended(event['request'], event['session'])

