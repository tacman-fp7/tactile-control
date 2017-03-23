
event_table_main = {
    See       = "e_exit",
    Let	      = "e_let",
    Forget    = "e_forget",
}

event_table_classify = {
    I         = "e_i_will",
    Try       = "e_try",
    Skip      = "e_skip",
    Here      = "e_here",
}

event_table_train = {
    Do        = "e_again",
    Discard   = "e_discard",
    Finished    = "e_train",
}


interact_fsm = rfsm.state{
   ----------------------------------
   -- state SUB_MENU_INITIAL       --
   ----------------------------------
    SUB_MENU_INITIAL = rfsm.state{
        entry=function()
          print("in substate MENU initial : waiting for speech command!")
        end,

        doo = function()
        while true do
        result = SM_Reco_Grammar(speechRecog_port, grammar_initial)
            print("MENU INITIAL received REPLY: ", result:toString() )
            local cmd =  result:get(1):asString()
            rfsm.send_events(fsm, event_table_main[cmd])
            rfsm.yield(true)
        end
    end
    },

    SUB_MENU_CLASSIFY = rfsm.state{
    entry=function()
        print("in substate MENU after reach : waiting for speech command!")
        end,

        doo = function()
        while true do
        result = SM_Reco_Grammar(speechRecog_port, grammar_classify)
            print("MENU reach received REPLY: ", result:toString() )
            local cmd =  result:get(1):asString()
            rfsm.send_events(fsm, event_table_classify[cmd])
            rfsm.yield(true)
        end
    end
    },

    SUB_MENU_TRAIN = rfsm.state{
    entry=function()
        print("in substate MENU after reach : waiting for speech command!")
        end,

        doo = function()
        while true do
        result = SM_Reco_Grammar(speechRecog_port, grammar_train)
            print("MENU reach received REPLY: ", result:toString() )
            local cmd =  result:get(1):asString()
            rfsm.send_events(fsm, event_table_train[cmd])
            rfsm.yield(true)
        end
    end
    },

   ----------------------------------
   -- states main                  --
   ----------------------------------

    SUB_EXIT = rfsm.state{
    entry=function()
        speak(ispeak_port, "Ok, bye bye")
        rfsm.send_events(fsm, 'e_menu_done')
    end
    },

    SUB_LET = rfsm.state{
        entry=function()
            local b = reach_out(manager_port)
            speak(ispeak_port,"Ok, Give me an object")
            local ret = b:get(0):asString()
        end
    },

    SUB_FORGET = rfsm.state{
        entry=function()
        --to be completed upon confirmation of possibility
        end
    },

    ----------------------------------
    -- classify  states             --
    ----------------------------------

    SUB_SKIP = rfsm.state{
        entry=function()
            speak(ispeak_port,"Ok returning back home")
            local b = reach_back(manager_port)
        end
    },

    SUB_HERE = rfsm.state{
        entry=function()
            speak(ispeak_port,"Ok") -- speak(ispeak_port,"Ok thank you")
            local b = grasp_object(manager_port)
            local ret = b:get(0):asString()
            print("Reply from grasp is", cmd)
        end
    },

    SUB_TRY = rfsm.state{
        entry=function()
            speak(ispeak_port,"Ok") -- speak(ispeak_port,"Ok, I will try it again")
            local b = grasp_object(manager_port)
            local ret = b:get(0):asString()
            print("Reply from grasp is", cmd)
        end
    },

    SUB_TEACH = rfsm.state{
        entry=function()
            speak(ispeak_port,"Ok, Tell me the name of the object")
            local objectcmd = SM_Reco_Grammar(speechRecog_port, grammar_object)
            print("received object REPLY: ", objectcmd:toString() )

            local skipcmd = objectcmd:get(1):asString()
            objectName = objectcmd:get(7):asString()

            print("received object Name: ", objectName )
            print("received skipcmd Name: ", skipcmd )

            if skipcmd == "Skip" then
                speak(ispeak_port,"Ok, Skipping")
                rfsm.send_events(fsm, "e_try")
            else
                local learn = learn_object(manager_port, objectName)
                speak(ispeak_port,"Ok, Let me have a look at the " .. objectName )
                local b = grasp_object(manager_port)
                local ret = b:get(0):asString()
                print("Reply from grasp is", cmd)
            end
        end
    },

    ----------------------------------
    -- train  states                --
    ----------------------------------

    SUB_AGAIN = rfsm.state{
        entry=function()
            speak(ispeak_port,"Ok") -- speak(ispeak_port,"Ok, Let me have another look at the " .. objectName )
            local b = grasp_object(manager_port)
        end
    },

    SUB_TRAIN = rfsm.state{
        entry=function()
            local b = process_object(manager_port)
            speak(ispeak_port,"Ok, I have learnt the " .. objectName .. ", give me an object" )
        end
    },

    SUB_DISCARD = rfsm.state{
        entry=function()
            speak(ispeak_port,"Ok, I will discard the last grasp" )
            local b = discard_object(manager_port)
        end
    },

   ----------------------------------
   -- state transitions            --
   ----------------------------------

    rfsm.trans{ src='initial', tgt='SUB_MENU_INITIAL'},
    rfsm.transition { src='SUB_MENU_INITIAL', tgt='SUB_EXIT', events={ 'e_exit' } },

    rfsm.transition { src='SUB_MENU_INITIAL', tgt='SUB_FORGET', events={ 'e_forget' } },
    rfsm.transition { src='SUB_FORGET', tgt='SUB_MENU_INITIAL', events={ 'e_done' } },

    rfsm.transition { src='SUB_MENU_INITIAL', tgt='SUB_LET', events={ 'e_let' } },
    rfsm.transition { src='SUB_LET', tgt='SUB_MENU_CLASSIFY', events={ 'e_done' } },

    ----------------------------------------------------------------------------------------

    rfsm.transition { src='SUB_MENU_CLASSIFY', tgt='SUB_HERE', events={ 'e_here' } },
    rfsm.transition { src='SUB_HERE', tgt='SUB_MENU_CLASSIFY', events={ 'e_done' } },

    rfsm.transition { src='SUB_MENU_CLASSIFY', tgt='SUB_TRY', events={ 'e_try' } },
    rfsm.transition { src='SUB_TRY', tgt='SUB_MENU_CLASSIFY', events={ 'e_done' } },

    rfsm.transition { src='SUB_MENU_CLASSIFY', tgt='SUB_SKIP', events={ 'e_skip' } },
    rfsm.transition { src='SUB_SKIP', tgt='SUB_MENU_INITIAL', events={ 'e_done' } },

    rfsm.transition { src='SUB_MENU_CLASSIFY', tgt='SUB_TEACH', events={ 'e_i_will' } },
    rfsm.transition { src='SUB_TEACH', tgt='SUB_MENU_CLASSIFY', events={ 'e_try' } },
    rfsm.transition { src='SUB_TEACH', tgt='SUB_MENU_TRAIN', events={ 'e_done' } },

    ----------------------------------------------------------------------------------------

    rfsm.transition { src='SUB_MENU_TRAIN', tgt='SUB_AGAIN', events={ 'e_again' } },
    rfsm.transition { src='SUB_AGAIN', tgt='SUB_MENU_TRAIN', events={ 'e_done' } },

    rfsm.transition { src='SUB_MENU_TRAIN', tgt='SUB_DISCARD', events={ 'e_discard' } },
    rfsm.transition { src='SUB_DISCARD', tgt='SUB_MENU_TRAIN', events={ 'e_done' } },

    rfsm.transition { src='SUB_MENU_TRAIN', tgt='SUB_TRAIN', events={ 'e_train' } },
    rfsm.transition { src='SUB_TRAIN', tgt='SUB_MENU_CLASSIFY', events={ 'e_done' } },
}
