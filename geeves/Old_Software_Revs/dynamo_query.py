# File can query dynamodb to find information if given a first name

from __future__ import print_function # Python 2/3 compatibility
import json # JSON encoder and decoder
import boto3 # Amazon Web Services SDK for Python
import decimal # Capability to use Python decimal function
from boto3.dynamodb.conditions import Key, Attr
from botocore.exceptions import ClientError

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

# Reference the appropriate AWS service
dynamodb = boto3.resource(
	'dynamodb',
	aws_access_key_id=ACCESS_KEY,
	aws_secret_access_key=SECRET_KEY,
)

table = dynamodb.Table('IDO_Studio')
username = 'charging'

response = table.query(
	TableName='IDO_Studio',
	IndexName='First_Name-Last_Name-index',
	KeyConditionExpression=Key('First_Name').eq(username)
)

for i in response['Items']:
	print(i['Office'], i['First_Name'], i['Last_Name'])


