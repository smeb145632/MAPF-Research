import json

with open('/mnt/f/MAPF/LeagueOfRobotRunners-MAPF/output_eecbs_30.json') as f:
    d = json.load(f)

print("plannerTimes (per timestep, in seconds):")
times = d.get('plannerTimes', [])
print("Count:", len(times))
if times:
    print("First 10:", times[:10])
    print("Last 10:", times[-10:])
    print("Total time:", sum(times))
    print("Max single step:", max(times))
    print("Min single step:", min(times))
    print("Avg:", sum(times)/len(times))
