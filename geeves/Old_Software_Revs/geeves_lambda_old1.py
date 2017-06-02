"""
# GE.EVES Office Robot project
# Program to determine if a name spoken to Alexa is present in a dynamoDB database
# If there is a match then the dynamoDB entry is modified to signify to 
# the GE.EVES Robot that it should proceed to the corresponding coordinates

from __future__ import print_function # Python 2/3 compatibility
import json # JSON encoder and decoder
import boto3 # Amazon Web Services SDK for Python
import decimal # Capability to use Python decimal function
from boto3.dynamodb.conditions import Key, Attr
from botocore.exceptions import ClientError

# Access keys necessary to query and modify dynamoDB
ACCESS_KEY='AKIAJE3KLYOD2MKHQFCA'
SECRET_KEY='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'

# Alexa Skill ID
alexa_skill_id = 'amzn1.ask.skill.bfcd7d3d-370e-475c-b926-4af489a68b14'

# Define variables that will be used globally
first_name_alexa = 'Name'
office_id = 'OfficeID'
dynamodb = boto3.resource(
	'dynamodb',
	aws_access_key_id=ACCESS_KEY,
	aws_secret_access_key=SECRET_KEY,
	)
table_name = 'IDO_Studio'
table = dynamodb.Table(table_name)
index_name = 'First_Name-Last_Name-index'
table_first_name = 'John'
table_last_name = 'Doe'

"""

# Helper class to convert a DynamoDB item to JSON.
class DecimalEncoder(json.JSONEncoder):
	def default(self, o):
	  if isinstance(o, decimal.Decimal):
	      if o % 1 > 0:
		return float (o)
	      else:
		return int (o)
	  return super(DecimalEncoder, self).default(o)


def lambda_handler(event, context):

    # Confirm event is for Alexa skill defined for GE.EVES
"""
    if (event["session"]["application"]["applicationId"] !=
            alexa_skill_id):
        raise ValueError("Invalid Application ID")
"""
    
    if event["session"]["new"]:
        on_session_started({"requestId": event["request"]["requestId"]}, event["session"])

    # Determine the type of event coming from Alexa

    if event["request"]["type"] == "LaunchRequest":
        return on_launch(event["request"], event["session"])
    elif event["request"]["type"] == "IntentRequest":
        return on_intent(event["request"], event["session"])
    elif event["request"]["type"] == "SessionEndedRequest":
        return on_session_ended(event["request"], event["session"])
"""
def on_session_started(session_started_request, session):
    return get_welcome_response()
"""

"""
def on_launch(launch_request, session):
    return get_welcome_response()
"""

"""
def on_intent(intent_request, session):
    intent = intent_request["intent"]
    intent_name = intent_request["intent"]["name"]

    if intent_name == "FindUser":
        return find_user(intent)
    elif intent_name == "YesIntent":
	return confirmed_user(intent)
    elif intent_name == "NoIntent":
	return wrong_user(intent)
    elif intent_name == "AMAZON.HelpIntent":
        return get_welcome_response()
    elif intent_name == "AMAZON.CancelIntent" or intent_name == "AMAZON.StopIntent":
        return handle_session_end_request()
    else:
        raise ValueError("Invalid intent")
"""

"""
def on_session_ended(session_ended_request, session):
    return handle_session_end_request()
"""

"""
def handle_session_end_request():
    card_title = "GE.EVES:  Come back soon!"
    speech_output = "Thank you for visiting eye-dee-oh.  See you next time!"
    should_end_session = True

    return build_response({}, build_speechlet_response(card_title, speech_output, None, should_end_session))
"""

"""
def get_welcome_response():
    session_attributes = {}
    card_title = "Welcome to the Industrial Design Studio!"
    speech_output = "Welcome to the industrial design studio. " \
		    "I'm Jeeves. " \
            	    "I can help guide you to the office of anyone that works here. " \
                    "Just tell me the first and last name of the person you're seeking. " 
    reprompt_text = "Sorry, I didn't catch that. " \
		    "Please repeat the first and last name of the person you need to find. "
    should_end_session = False

    return build_response(session_attributes, build_speechlet_response(
        card_title, speech_output, reprompt_text, should_end_session))
"""

def no_user_found():
    session_attributes = {}
    card_title = "Sorry, I couldn't find a match."
    speech_output = "Sorry, I couldn't find a match. " \
		    "Please repeat the name of the person you're seeking or say stop to cancel. "
    reprompt_text = "I'm not sure whose office you want to find. " \
            "Please repeat the name of the person you're seeking or say stop to cancel. "
    should_end_session = False

    return build_response(session_attributes, build_speechlet_response(
        card_title, speech_output, reprompt_text, should_end_session))

def user_found():
    session_attributes = {}
    card_title = "I found " + first_name_alexa + "'s office.  Follow me!"
    speech_output = "Great! " \
	            "I know just where to find " + first_name_alexa +"'s office. " \
	            "Follow me and I'll take you there. "
    should_end_session = True

    return build_response(session_attributes, build_speechlet_response(
        card_title, speech_output, None, should_end_session))

def ask_for_confirmation():
    session_attributes = {}
    card_title = "Are you looking for " + table_first_name + table_last_name + "'s office? " 
    speech_output = "Just to confirm, are you looking for " + table_first_name + table_last_name + "'s office? "
    reprompt_text = "Sorry, I didn't hear your response. " \
            "Are you looking for " + table_first_name + table_last_name + "'s office? "\
	    "Say yes to confirm, or no if I have the wrong name."
    should_end_session = False

    return build_response(session_attributes, build_speechlet_response(
        card_title, speech_output, None, should_end_session))

def confirmed_user_intent(intent):

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

    # Run routine specifying output once a user is found and the database entry is updated
    return user_found()

def wrong_user(intent):

    return no_user_found()

def db_names():

    dbnames = table.query(
    TableName = table_name,
    IndexName = index_name,
    KeyConditionExpression=Key('Office').eq(office_id),
    )

    global table_first_name = dbnames[u'Items'][0][u'First_Name']
    global table_last_name = dbnames[u'Items'][0][u'Last_Name']
 
    return ask_for_confirmation()

def find_user(intent):
    session_attributes = {}

    # Get First Name from Intent and create first name variations for searching

    if "First_Name" in intent["slots"]:
	global first_name_alexa
        first_name_alexa = intent["slots"]["First_Name"]["value"]
        first_name_alexa_lc = first_name_alexa.lower()
	first_name_two_letters = first_name_alexa_lc[0:2]
	first_name_first_letter = first_name_alexa_lc[0:1]

    # Get Last Name from Intent and create last name variations for searching

    if "Last_Name" in intent["slots"]:
	global last_name_alexa
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

            dbresponse = table.query(
            TableName = table_name,
	    IndexName = index_name,
	    KeyConditionExpression=Key('First_Name').eq(first_name_alexa_lc) & Key('Last_Name').begins_with(last_name_two_letters),
	    )

	    if dbresponse['Count'] == 0:

		dbresponse = table.query(
		TableName = table_name,
		IndexName = index_name,
		KeyConditionExpression=Key('First_Name').begins_with(first_name_two_letters) & Key('Last_Name').begins_with(last_name_two_letters),
		)

		if dbresponse['Count'] == 0:

		    dbresponse = table.query(
		    TableName = table_name,
		    IndexName = index_name,
		    KeyConditionExpression=Key('First_Name').begins_with(first_name_two_letters) & Key('Last_Name').begins_with(last_name_first_letter),
		    )

		    if dbresponse['Count'] == 0:

			dbresponse = table.query(
			TableName = table_name,
			IndexName = index_name,
			KeyConditionExpression=Key('First_Name').begins_with(first_name_first_letter) & Key('Last_Name').begins_with(last_name_two_letters),
			)

			if dbresponse['Count'] == 0:

			    dbresponse = table.query(
			    TableName = table_name,
			    IndexName = index_name,
			    KeyConditionExpression=Key('First_Name').eq(first_name_alexa_lc),
			    )
	
			    if dbresponse['Count'] == 0:

				dbresponse = table.query(
			        TableName = table_name,
			        IndexName = index_name,
			        KeyConditionExpression=Key('Last_Name').eq(last_name_alexa_lc),
			        )

				if dbresponse['Count'] == 0:

			    	    dbresponse = table.query(
			    	    TableName = table_name,
			    	    IndexName = index_name,
			    	    KeyConditionExpression=Key('First_Name').begins_with(first_name_first_letter) & Key('Last_Name').begins_with(last_name_first_letter),
			    	    )

			    	    if dbresponse['Count'] == 0:

			        	return no_user_found()

	if dbresponse['Count'] == 1:

	    # Find Office ID in database associated with the first name
	    global office_id = dbresponse[u'Items'][0][u'Office']
	    
            return db_names()

	else:

  	    return multi_user_found()

    elif "First_Name" in intent["slots"] and not "Last_Name" in intent["slots"]:

	dbresponse = table.query(
        TableName = table_name,
	IndexName = index_name,
	KeyConditionExpression=Key('First_Name').eq(first_name_alexa_lc),
	)

	if dbresponse['Count'] == 0:

            dbresponse = table.query(
            TableName = table_name,
	    IndexName = index_name,
	    KeyConditionExpression=Key('First_Name').begins_with(first_name_two_letters),
	    )

	    if dbresponse['Count'] == 0:

		dbresponse = table.query(
		TableName = table_name,
		IndexName = index_name,
		KeyConditionExpression=Key('First_Name').begins_with(first_name_first_letter),
	    	)

		if dbresponse['Count'] == 0:

		    return no_user_found()

	if dbresponse['Count'] == 1:

	    # Find Office ID in database associated with the first name
	    global office_id = dbresponse[u'Items'][0][u'Office']
	    
            return db_names()

	else:

  	    return multi_user_found()

    elif "Last_Name" in intent["slots"] and not "First_Name" in intent["slots"]:

	dbresponse = table.query(
        TableName = table_name,
	IndexName = index_name,
	KeyConditionExpression=Key('Last_Name').eq(last_name_alexa_lc),
	)

	if dbresponse['Count'] == 0:

            dbresponse = table.query(
            TableName = table_name,
	    IndexName = index_name,
	    KeyConditionExpression=Key('Last_Name').begins_with(last_name_two_letters),
	    )

	    if dbresponse['Count'] == 0:

		dbresponse = table.query(
		TableName = table_name,
		IndexName = index_name,
		KeyConditionExpression=Key('First_Name').begins_with(last_name_first_letter),
	    	)

		if dbresponse['Count'] == 0:

		    return no_user_found()

	if dbresponse['Count'] == 1:

	    # Find Office ID in database associated with the first name
	    global office_id = dbresponse[u'Items'][0][u'Office']
	    
            return db_names()

	else:

  	    return multi_user_found()

    else:

	return no_user_found()

"""
def build_speechlet_response(title, output, reprompt_text, should_end_session):
    return {
        "outputSpeech": {
            "type": "PlainText",
            "text": output
        },
        "card": {
            "type": "Simple",
            "title": title,
            "content": output
        },
        "reprompt": {
            "outputSpeech": {
                "type": "PlainText",
                "text": reprompt_text
            }
        },
        "shouldEndSession": should_end_session
    }

def build_response(session_attributes, speechlet_response):
    return {
        "version": "1.0",
        "sessionAttributes": session_attributes,
        "response": speechlet_response
    }

"""
