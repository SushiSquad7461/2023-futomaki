import json

BASE_DIR = "C:\\Users\\rsidh\\Programs\\Robotics\\2023-temaki\\src\\main\\deploy\\pathplanner\\"

pathName = input("enter the name of the path:")

oldPathFile = open(BASE_DIR+pathName)

path = json.load(oldPathFile)

for waypoint in path["waypoints"]:
    waypoint["anchorPoint"]["x"] = 16.5-waypoint["anchorPoint"]["x"]
    if waypoint["prevControl"] != None:
        waypoint["prevControl"]["x"] = 16.5 - waypoint["prevControl"]["x"]
    if waypoint["nextControl"] != None:
        waypoint["nextControl"]["x"] = 16.5 - waypoint["nextControl"]["x"]
    waypoint["holonomicAngle"] = 180.0 - waypoint["holonomicAngle"]
    if waypoint["holonomicAngle"] > 180.0:
        waypoint["holonomicAngle"] -= 360.0

if pathName[:3] != "Red":
    newPathName = "Red_" + pathName
else:
    newPathName = pathName[4:]

newPathFile = open(f"{BASE_DIR}{newPathName}", "x")

newPathFile.write(json.dumps(path))