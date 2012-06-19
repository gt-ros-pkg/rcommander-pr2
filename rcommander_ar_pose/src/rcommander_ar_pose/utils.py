#import tf
#self.broadcaster = tf.TransformBroadcaster()
#self.tf_listener = tf.TransformListener()
import re

def find_ar_frames(tf_listener):
    all_frames = tf_listener.getFrameStrings()
    ar_transforms = []
    fourfour = re.compile('^/4x4_\d+')
    #match AR tool kit frames
    for T in all_frames:
        mobj = fourfour.match(T)
        if mobj != None and not (T in ar_transforms):
            ar_transforms.append(T)
    return ar_transforms

