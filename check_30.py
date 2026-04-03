import json

with open('/mnt/f/MAPF/LeagueOfRobotRunners-MAPF/output_eecbs_30.json') as f:
    d = json.load(f)

print("numTaskFinished:", d.get('numTaskFinished'))
print("makespan:", d.get('makespan'))
print("sumOfCost:", d.get('sumOfCost'))
print("teamSize:", d.get('teamSize'))
print("events count:", len(d.get('events', [])))
print("tasks count:", len(d.get('tasks', [])))
print()
print("Last 5 events:")
for e in d.get('events', [])[-5:]:
    print(e)
