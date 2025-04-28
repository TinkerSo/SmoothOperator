import requests

headers = { 'Content-Type': 'text/plain' }
requests.post("http://10.193.24.226:3000/api/ros", data="Goal Reached", headers=headers)
