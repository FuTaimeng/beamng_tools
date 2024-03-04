local M = {}
local wheelCache = {}

local _log = log
local function log(level, msg)
  _log(level, 'vehicleEngineCode', msg)
end

local function onSocketMessage(request)
    local msgType = 'handle' .. request['type']
    local handler = M[msgType]
    if handler ~= nil then
        handler(request)
    else
        log('E', 'handler does not exist: ' .. msgType)
    end
end

local function onExtensionLoaded()
    wheelCache = {}
    for i = 0, wheels.wheelRotatorCount - 1 do
        local wheel = wheels.wheelRotators[i]
        wheelCache[wheel.name] = wheel
    end
end

local function handleGetWheelSpeeds(request)
    local response = {}
    for name, wheel in pairs(wheelCache) do
        response[name] = wheel.wheelSpeed
    end
    request:sendResponse(response)
end

M.onExtensionLoaded = onExtensionLoaded
M.onSocketMessage = onSocketMessage
M.handleGetWheelSpeeds = handleGetWheelSpeeds

return M