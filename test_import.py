import sys
sys.path.append('./build')
import MAPF
print("MAPF imported successfully")
sys.path.remove('./build')
sys.path.append('./python')
import EECBSPlanner
print("EECBSPlanner imported successfully")
