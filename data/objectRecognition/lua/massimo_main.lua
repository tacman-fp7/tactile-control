#!/usr/bin/lua

require("yarp")
require("rfsm")
--require("iol_funcs")

yarp.Network()

-------
shouldExit = false

-- initilization
ispeak_port = yarp.BufferedPortBottle()
speechRecog_port = yarp.Port()
manager_port = yarp.Port()

-- defining of objects
objects = {"Soccerball", "Object", "reviewobject", "Final review meeting object", "Bottle", "Box"}

-- defining speech grammar for initial menu
grammar_initial = "Let start | See you soon | Forget #Object | Forget all objects"

-- defining speech grammar for classifying
grammar_classify = "Here you go | Try it again | Skip it | I will teach you a new object | Have another look at the #Object "

-- defining speech grammar for teaching a new object
grammar_object = "This is a #Object | Skip it"

-- defining speech grammar for training
grammar_train = "Do it again | Finished | Discard the last grasp"



-- load state machine model and initalize it
rf = yarp.ResourceFinder()
rf:setDefaultContext("massimo/lua")
rf:configure(arg)
fsm_file = rf:findFile("massimo_root_fsm.lua")
fsm_model = rfsm.load(fsm_file)
fsm = rfsm.init(fsm_model)
rfsm.run(fsm)

repeat
    rfsm.run(fsm)
    yarp.Time_delay(0.1)
until shouldExit ~= false

print("finishing")
-- Deinitialize yarp network
yarp.Network_fini()

