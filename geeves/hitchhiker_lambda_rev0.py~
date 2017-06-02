# Turtlebot office concierge project
# Program to determine if a name spoken to Alexa is present in a dynamoDB database
# If there is a match then the dynamoDB entry is modified to signify the Turtlebot should proceed to the corresponding coordinates

from __future__ import print_function # Python 2/3 compatibility
import json # JSON encoder and decoder
import boto3 # Amazon Web Services SDK for Python
import decimal # Capability to use Python decimal function
from boto3.dynamodb.conditions import Key, Attr
from botocore.exceptions import ClientError

# Access keys necessary to query and modify dynamoDB
ACCESS_KEY='AKIAJE3KLYOD2MKHQFCA'
SECRET_KEY='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'

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

    # Confirm event is for Alexa skill defined for turtlebot concierge (hitchhiker)

    if (event["session"]["application"]["applicationId"] !=
            "amzn1.ask.skill.bfcd7d3d-370e-475c-b926-4af489a68b14"):
        raise ValueError("Invalid Application ID")
    
    if event["session"]["new"]:
        on_session_started({"requestId": event["request"]["requestId"]}, event["session"])

    # Determine the type of event coming from Alexa

    if event["request"]["type"] == "LaunchRequest":
        return on_launch(event["request"], event["session"])
    elif event["request"]["type"] == "IntentRequest":
        return on_intent(event["request"], event["session"])
    elif event["request"]["type"] == "SessionEndedRequest":
        return on_session_ended(event["request"], event["session"])

def on_session_started(session_started_request, session):
    print "Starting new session."

def on_launch(launch_request, session):
    return get_welcome_response()

def on_intent(intent_request, session):
    intent = intent_request["intent"]
    intent_name = intent_request["intent"]["name"]

    if intent_name == "GuideMe":
        return guide_me(intent)
    elif intent_name == "AMAZON.HelpIntent":
        return get_welcome_response()
    elif intent_name == "AMAZON.CancelIntent" or intent_name == "AMAZON.StopIntent":
        return handle_session_end_request()
    else:
        raise ValueError("Invalid intent")

def on_session_ended(session_ended_request, session):
    print "Ending session."
    # Cleanup goes here...

def handle_session_end_request():
    card_title = "IDO Hitchhiker.  Come back soon!"
    speech_output = "Thank you for visiting the i. d. o. studio.  See you next time!"
    should_end_session = True

    return build_response({}, build_speechlet_response(card_title, speech_output, None, should_end_session))

def get_welcome_response():
    session_attributes = {}
    card_title = "Welcome to the IDO Studio!"
    speech_output = "Welcome to the i. d. o. studio. " \
		    "I'm hitchhiker. " \
                    "I can help guide you to the office of anyone working in the studio. " \
                    "To start, please phrase your request as follows." \
		    "Show me to the office of first name last name."
    reprompt_text = "Sorry, I didn't understand your request. " \
		    "But, I will be happy to try again. " \
		    "Please phrase your request the same as given in the following example. " \
		    "Show me to the office of Jane Smith. "
    should_end_session = False
    return build_response(session_attributes, build_speechlet_response(
        card_title, speech_output, reprompt_text, should_end_session))

def guide_me(intent):
    session_attributes = {}

    # Determine if first name from Alexa matches a first name entry in dynamoDB

    if "First_Name" in intent["slots"]:
        first_name_alexa = intent["slots"]["First_Name"]["value"]
        first_name_alexa_lc = first_name_alexa.lower()
	
    # Reference the appropriate AWS service - dynamoDB
	
        dynamodb = boto3.resource(
	'dynamodb',
	aws_access_key_id=ACCESS_KEY,
	aws_secret_access_key=SECRET_KEY,
	)

    # Define the dyanamoDB table to search
	
	table = dynamodb.Table('IDO_Studio')

    # Query the table to determine if there is are any First_Name matches

	dbresponse = table.query(
	TableName='IDO_Studio',
	IndexName='First_Name-Last_Name-index',
	KeyConditionExpression=Key('First_Name').eq(first_name_alexa_lc)
	)

        # Determine the number of entries in the database that match the first name from Alexa.

        # If there is only 1 exact first name match then update that database entry.
        # This entry creates a signal for the robot to travel to that coordinate set.

	if dbresponse['Count'] == 1:
	    
	    # Find Office ID in database associated with the first name
	    office_id = dbresponse[u'Items'][0][u'Office']

### INPUT CODE TO MODIFY LOCATE_OFFICE ENTRY TO TRUE ###

	    card_title = "I found " + first_name_alexa + "'s office.  Follow me!"
            speech_output = "No problem." \
	                    "I know just where to find " + first_name_alexa +"'s office. " \
			    "Follow me and I'll take you there.
	    reprompt_text = ""

	    should_end_session = True

	elif dbresponse['Count'] == 0:

	    # Notify user that the name could not be found.

            speech_output = "I could not find a match for the name you provided. " \
		            "But, I'm happy to try again. " \
		            "Please phrase your request the same as given in the following example. " \
		            "Show me to the office of Jane Smith. "
            reprompt_text = "I'm not sure whose office you want to find. " \
                            "Please phrase your request the same as given in the following example. " \
		            "Show me to the office of Jane Smith. "
            should_end_session = False  

	else:

	    # Search for the exact match of first name and last name in database.
	
### INPUT CODE TO SEARCH AGAINST FIRST NAME AND LAST NAME, THEN DETERMINE IF THERE IS A MATCH ###	    	
	    

    return build_response(session_attributes, build_speechlet_response(
              card_title, speech_output, reprompt_text, should_end_session)) 


        if (station_code != "unkn"):
            card_title = "BART Departures from " + station_name.title()

            response = urllib2.urlopen(API_BASE + "/departures/" + station_code)
            station_departures = json.load(response)   

            speech_output = "Train departures from " + station_name + " are as follows: "
            for destination in station_departures["etd"]:
                speech_output += "Towards " + destination["destination"] + " on platform " + destination["estimate"][0]["platform"] + ". ";
                for estimate in destination["estimate"]:
                    if estimate["minutes"] == "Leaving":
                        speech_output += "Leaving now: "
                    elif estimate["minutes"] == "1":
                        speech_output += "In one minute: "
                    else:
                        speech_output += "In " + estimate["minutes"] + " minutes: "

                    speech_output += estimate["length"] + " car train. "

            reprompt_text = ""

    return build_response(session_attributes, build_speechlet_response(
        card_title, speech_output, reprompt_text, should_end_session))

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


