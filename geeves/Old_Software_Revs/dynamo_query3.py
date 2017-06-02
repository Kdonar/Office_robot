# Program can query dynamodb and determine if first name and last name combination has 0, 1, or >1 matches.

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

response1 = table.query(
	TableName='IDO_Studio',
	IndexName='First_Name-Last_Name-index',
	KeyConditionExpression=Key('First_Name').eq('charging') & Key('Last_Name').eq('base'),
)


if response1['Count'] == 1:
	matches = 1

elif response1['Count'] == 0:
	matches = 0
else:
	matches = 2

print (matches)



