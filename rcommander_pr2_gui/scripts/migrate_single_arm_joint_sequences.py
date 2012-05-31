import roslib; roslib.load_manifest('rcommander_pr2_gui')
import rcommander.graph_model as gm
import rcommander_pr2_gui.move_tool as mt
import sys
import pdb

print 'loading', sys.argv[1]
graph_model = gm.GraphModel.load(sys.argv[1])
states = graph_model.states_dict
for k in states.keys():
    state = states[k]
    if state.__class__ == mt.JointSequenceState:
        arm = state.arm
        for wp in state.joint_waypoints:
            wp['data']['arm'] = arm

        states[k] = mt.JointSequenceState(k, state.joint_waypoints)

print 'saving to', graph_model.document.get_filename()
graph_model.save(graph_model.document.get_filename())

