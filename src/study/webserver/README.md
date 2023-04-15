# Webserver for the CoMOTO Experiment

## Prereqs
```
pip install Flask requests
```

## Running the Webserver
```
python app.py
```

After starting the webserver, go to your web browser to open the web interface
```
http://127.0.0.1:5000
```

## Choosing Bolt Ordering to Display (call from Python)
Here is an example to request the web interface to display the bolt ordering (or wait) images
```python
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
```

