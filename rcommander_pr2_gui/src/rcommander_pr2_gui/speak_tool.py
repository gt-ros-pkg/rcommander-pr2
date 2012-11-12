import rospy
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import rcommander.tool_utils as tu

from tts_server.srv import *
import smach

## Tool to interface with the Festival text-to-speech engine.
class SpeakTool(tu.ToolBase):

    ## Default phrase
    DEFAULT_TEXT = 'hello world'

    ## Constructor
    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'speak', 'Speak', SpeakNode)
        self.available_voices = rospy.ServiceProxy('available_voices', AvailableVoices, persistent=True)

    ## Inherited
    def fill_property_box(self, pbox):
        formlayout = pbox.layout()
        self.text = QPlainTextEdit(pbox)
        self.voices_box = QComboBox(pbox)
        ret = self.available_voices()
        ret.voices.sort()
        for v in ret.voices:
            self.voices_box.addItem(v)

        self.reset()
        formlayout.addRow('&Voice', self.voices_box)
        formlayout.addRow('&Say', self.text)

    ## Inherited
    def new_node(self, name=None):
        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name
        
        txt = str(self.text.document().toPlainText())
        voice = str(self.voices_box.currentText())
        if voice == ' ':
            return None
        return SpeakNode(nname, txt, voice)

    ## Inherited
    def set_node_properties(self, my_node):
        self.document.setPlainText(my_node.text)
        idx = tu.combobox_idx(self.voices_box, my_node.voice)
        self.voices_box.setCurrentIndex(idx)

    ## Inherited
    def reset(self):
        self.document = QTextDocument(self.DEFAULT_TEXT)
        self.layout = QPlainTextDocumentLayout(self.document)
        self.document.setDocumentLayout(self.layout)
        self.text.setDocument(self.document)
        self.voices_box.setCurrentIndex(0)


class SpeakNodeSmach(smach.State): 

    ## Constructor
    # @param text Text to speak (string).
    # @param voice Voice to use (string).
    def __init__(self, text, voice):
        smach.State.__init__(self, outcomes = ['done'], input_keys = [], output_keys = [])
        self.text = text
        self.voice = voice

    ## Inherited
    def execute(self, userdata):
        self.say = rospy.ServiceProxy('say', Say)
        self.say(self.voice, self.text)
        return 'done'


class SpeakNode(tu.StateBase):

    ## Constructor
    def __init__(self, name, text, voice):
        tu.StateBase.__init__(self, name)
        self.text = text
        self.voice = voice

    ## Inherited
    def get_smach_state(self):
        return SpeakNodeSmach(self.text, self.voice)

