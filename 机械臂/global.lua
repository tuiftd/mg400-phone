-- 全局变量模块仅用于定义全局变量和模块函数，不能调用运动指令。
-- Version: Lua 5.3.5
local Err
local Socket
function start()
  Err, Socket = TCPCreate(false, "192.168.1.100", 7930)
  if Err==0 then
    TCPStart(Socket, 0)
  end
end

function send(text)
  TCPWrite(Socket,text)
end

function fen(msg,tp)
  local result={}
  string.gsub(msg,'[^'..tp..']+',function(w)
  table.insert(result,w)
  end
  )
  do return result end
end

function get()
  local x=350
  local y=0
  local r=0
  local x_goto=0
  local y_goto=0
  local r_goto=0
  local Aresult=nil
  local a={}
  local t={}
  local RecBuf
  local msg=""
  local result={}
  local msg2use={}
  print("ready 2 get msg\r")
  --send("rob ready")
  Err, RecBuf = TCPRead(Socket, 0, "string")
  msg=RecBuf.buf
  print("\r"..msg)
  msg2use = fen(msg,",")
  Aresult = msg2use[1]
  if Aresult~="over" then
    x=tonumber(msg2use[2])
    y=tonumber(msg2use[3])
    r=tonumber(msg2use[4])
    --print("\r"..x..","..y..","..r)
    x_goto=tonumber(msg2use[5])
    y_goto=tonumber(msg2use[6])
    r_goto=tonumber(msg2use[7])
    --print("\r"..x_goto..","..y_goto..","..r_goto)
  end
  local P1 = {coordinate= { x,y,-162.01,r},user=0, tool = 0}
  local P2 = {coordinate= { x_goto,y_goto,-150.06,r_goto},user=0, tool = 0}
  do return Aresult,P1,P2 end
end