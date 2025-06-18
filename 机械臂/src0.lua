-- Version: Lua 5.3.5
Option={SpeedL=50, AccL=20, Start=0, ZLimit=0, End=0,"SYNC=1"}
local Aresult=nil
start()
send("rob")
Jump(P1,Option)
while(1) do
  Aresult,Point,Point_goto=get()
  if Aresult~="over" then
    Jump(Point, Option)
    DO(2,ON)
    Wait(300)
    Jump(Point_goto, Option)
    DO(2,OFF)
    Wait(300)
    DO(1,ON)
    Wait(300)
    DO(1,OFF)
    Sync(1)
    send("go")
  else
    Jump(P1,Option)
  end
  
end