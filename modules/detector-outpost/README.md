# Detector of Outpost
## Basic Params:
At the beginning 30 seconds of the game, outpost is invincible status. 

So <big>**_pay attention_**</big> to shoot it **_after 30_** seconds.

Rotate speed: 0.4 round per second, namely 144 degrees per second.

Included angle between 2 armors: 120 degree, namely 2/3 rad.

## Basic Method:
Detect Armors for 0.84 seconds.

Push_back every armor into a vector, preparing data for getting ideal armor in the following.

**_[Pay attention]_** Reload operator '<' or '>' to directly compare armor's area in the vector. 

When detected for 0.84 seconds, getting the biggest one from the vector. 

Then returning its center point as the target point of the outpost, and it's okay to predict. 