#!/usr/bin/env python3
"""Print docs/controller.html to docs/controller.pdf with headless Firefox.

Needs only Firefox and its geckodriver (Ubuntu's Firefox snap ships both);
the maths is MathML, which Firefox renders natively. Stdlib only.

    python3 docs/build_pdf.py

Firefox as a snap can only read files under $HOME, so keep the repo there.
"""
import base64
import json
import os
import subprocess
import time
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
PORT = 4456


def call(method, path, body=None):
    req = urllib.request.Request(
        f"http://127.0.0.1:{PORT}{path}", method=method,
        data=None if body is None else json.dumps(body).encode(),
        headers={"Content-Type": "application/json"})
    return json.load(urllib.request.urlopen(req, timeout=120))["value"]


def driver_up():
    try:
        return call("GET", "/status").get("ready") is not None
    except OSError:
        return False


def main():
    if not driver_up():  # reuse a running geckodriver, else start one
        subprocess.Popen(["geckodriver", "--port", str(PORT)],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                         start_new_session=True)
        for _ in range(50):
            if driver_up():
                break
            time.sleep(0.2)
    sid = call("POST", "/session", {"capabilities": {"alwaysMatch": {
        "moz:firefoxOptions": {"args": ["-headless"]}}}})["sessionId"]
    try:
        call("POST", f"/session/{sid}/url",
             {"url": "file://" + os.path.join(HERE, "controller.html")})
        time.sleep(1.0)  # let images and SVG figures finish loading
        pdf = call("POST", f"/session/{sid}/print", {
            "background": True, "shrinkToFit": True,
            "page": {"width": 21.0, "height": 29.7},
            "margin": {"top": 1.8, "bottom": 1.8, "left": 2.0, "right": 2.0}})
        out = os.path.join(HERE, "controller.pdf")
        with open(out, "wb") as f:
            f.write(base64.b64decode(pdf))
        print("wrote", out)
    finally:
        call("DELETE", f"/session/{sid}")


if __name__ == "__main__":
    main()
