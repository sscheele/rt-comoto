import requests

myreq = {'which_img': "T2_human_bolt_ordering.png"}
requests.post("http://127.0.0.1:5000/get-which-img", json=myreq)