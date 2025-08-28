# Drone Control Hackathon

> The Helsing hackathon challenge for controlling a drone in adverse conditions.

## Challenge Prompt

Testing drones is hard enough even with a facility like ASTA. To whet your appetite in between testing times, this drone 
challenge will _simulate_ a real environment with external negative factors such as wind and battery. 
Your goal is to create a control algorithm that pilots your drone into a precise point in space, despite these adverse conditions.

Can you make an algorithm that's better than everyone else's?

## Getting started

To get setup, first clone this repo:
```bash
git clone git@github.com:helsing-ai/edth-copenhagen-drone-control.git
```

If you haven't already, you'll need to sign up to the challenge. Visit `http://172.104.137.51:10300/login` and insert your 
email address. You'll receive an email with further details on the challenge and a token.

Once you have signed up and received your token, you can register for the challenge. Choose the name of your drone that
will be displayed in the leaderboard, and replace TOKEN and DRONE_NAME in the following command, then run it:
```
curl -XPOST -H 'content-type: application/json' -H 'Authorization:Bearer TOKEN' -d '{"drone_name": "DRONE_NAME"}' http://172.104.137.51:10300/challenge
```

There will be a big screen in the room with a 3D view of all current attempts to this challenge. This will also contain 
the leaderboard, so you can see who's in the lead!

Otherwise, check out `http://172.104.137.51:10400/static/index.html` from your own screen.

## Using the template (optional)

Now you are all set to start hacking! In this repository, we provide two templates for you to get started on the challenge.
They aim to take away some of the boilerplate you'd have to write to interact with the simulation server. We've done this in
Rust and Python, but of course you are free to implement everything from scratch or even in a different language, as long
as it supports protobuf!

This project uses [uv](https://docs.astral.sh/uv/) to manage dependencies; follow the
[uv installation guide](https://docs.astral.sh/uv/getting-started/installation/) if you don't already have it installed.
Then create the virtual environment with:
```bash
uv sync
```

You can run the example Python script from the repository's root directory with:
```
uv run python py-src/drone/drone.py
```
If you are getting `module not found` errors, try running `PYTHONPATH=./py-src uv run python py-src/drone/drone.py` instead.

If you want to work with Rust, you simply have to run `cargo run` from the repository's root which will execute the `src/main.rs` file.
