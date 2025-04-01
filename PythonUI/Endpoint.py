import requests

payload = {"command": "Goal Reached"}
response = requests.post("http://10.193.24.226:3000/api/ros", json=payload)
print("Response:", response.text)
