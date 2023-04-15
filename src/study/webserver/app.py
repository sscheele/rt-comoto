#!/usr/bin/env python3
from flask import Flask, render_template, request, jsonify
import os

"""
Request the bolt ordering imges(how to call)
>> import requests

# Trial 1
>> which_img = {'which_img': "T1_human_bolt_ordering.png"} #trial 1
>> requests.post("http://127.0.0.1:5000/get-which-img", json=which_img)
>> which_img["which_img"] = "wait.png"
>> requests.post("http://127.0.0.1:5000/get-which-img", json=which_img)

# Trial 2
>> which_img["which_img"] = "T2_human_bolt_ordering.png"
>> requests.post("http://127.0.0.1:5000/get-which-img", json=which_img)
>> which_img["which_img"] = "wait.png"
>> requests.post("http://127.0.0.1:5000/get-which-img", json=which_img)
# Trial 3
>> which_img["which_img"] = "T3_human_bolt_ordering.png"
>> requests.post("http://127.0.0.1:5000/get-which-img", json=which_img)
>> which_img["which_img"] = "wait.png"
>> requests.post("http://127.0.0.1:5000/get-which-img", json=which_img)

>> which_img["which_img"] = "T4_human_bolt_ordering.png"
>> requests.post("http://127.0.0.1:5000/get-which-img", json=which_img)
>> which_img["which_img"] = "wait.png"
>> requests.post("http://127.0.0.1:5000/get-which-img", json=which_img)
"""
app = Flask(__name__)

#index route
@app.route("/", methods=["GET"])
def index_route():
    return render_template("index.html", which_img=which_img)

# image post route
@app.route("/get-which-img", methods=["POST"])
def get_which_img_post_route():
    global which_img
    data=request.get_json()
    which_img = data["which_img"]
    return jsonify({"which_img":which_img})

@app.route("/get-which-img", methods=["GET"])
def get_which_img_get_route():
    return jsonify({"which_img":which_img})


if __name__ == "__main__":
    which_img = "wait.png"
    app.run("0.0.0.0", port=5000)