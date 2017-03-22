
function speak(port, str)
   local wb = port:prepare()
    wb:clear()
    wb:addString(str)
    port:write()
    yarp.Time_delay(1.0)
end

----------------------------------
-- functions MOTOR              --
----------------------------------

function reach_out(port)
    local wb = yarp.Bottle()
    local reply = yarp.Bottle()
    wb:clear()
    wb:addString("arm")
    wb:addString("up")
    port:write(wb,reply)
    return reply
end

function reach_back(port)
    local wb = yarp.Bottle()
    local reply = yarp.Bottle()
    wb:clear()
    wb:addString("arm")
    wb:addString("down")
    port:write(wb,reply)
    return reply
end

function grasp_object(port)
    local wb = yarp.Bottle()
    local reply = yarp.Bottle()
    wb:clear()
    wb:addString("grasp")
    port:write(wb,reply)
    return reply
end

----------------------------------
-- functions LEARN              --
----------------------------------

function learn_object(port, obj)
    local wb = yarp.Bottle()
    local reply = yarp.Bottle()
    wb:clear()
    wb:addString("ml")
    wb:addString("learn_new_object")
    wb:addString(obj)
    port:write(wb,reply)
    return reply
end

function process_object(port)
    local wb = yarp.Bottle()
    local reply = yarp.Bottle()
    wb:clear()
    wb:addString("ml")
    wb:addString("process")
    port:write(wb,reply)
    return reply
end

function discard_object(port)
    local wb = yarp.Bottle()
    local reply = yarp.Bottle()
    wb:clear()
    wb:addString("ml")
    wb:addString("discard")
    port:write(wb,reply)
    return reply
end


----------------------------------
-- functions SPEECH             --
----------------------------------

function IH_Expand_vocab(port, objects)
    local wb = yarp.Bottle()
    local reply = yarp.Bottle()

    wb:addString("name")
    for key, word in pairs(objects) do
        wb:addString(word)
    end
    port:write(wb,reply)
    
    local rep  =  reply:get(0):asString()
    
    if rep == "ack" then
        for k in pairs (objects) do
            objects[k] = nil
        end
        for i=1, reply:size()-1 do
            objects[i] = reply:get(i):asString()
            print("objects are: ", objects[i])
        end
    else
        print("Was not able to set the new vocabulary: ", reply:get(0):asString() )
    end
   return rep
end

function SM_RGM_Expand(port, vocab, word)
    local wb = yarp.Bottle()
    local reply = yarp.Bottle()
    wb:clear()
    wb:addString("RGM")
    wb:addString("vocabulory")
    wb:addString("add")
    wb:addString(vocab)
    wb:addString(word)
    port:write(wb)
    --port:write(wb,reply)
    return "OK" --reply:get(1):asString()
end

function SM_Expand_asyncrecog(port, gram)
    local wb = yarp.Bottle()
    local reply = yarp.Bottle()
    wb:clear()
    wb:addString("asyncrecog")
    wb:addString("addGrammar")
    wb:addString(gram)
    port:write(wb,reply)
end

function SM_Reco_Grammar(port, gram)
    local wb = yarp.Bottle()
    local reply = yarp.Bottle()
    wb:clear()
    wb:addString("recog")
    wb:addString("grammarSimple")
    wb:addString(gram)
    port:write(wb,reply)
    return reply
end

function SM_RGM_Expand_Auto(port, vocab)
    local wb = yarp.Bottle()
    local reply = yarp.Bottle()
    wb:clear()
    wb:addString("RGM")
    wb:addString("vocabulory")
    wb:addString("addAuto")
    wb:addString(vocab)
    port:write(wb,reply)
    return reply:get(1):asString()
end

