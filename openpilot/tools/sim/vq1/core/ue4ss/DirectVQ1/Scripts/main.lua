local TAG = "[DirectVQ1]"
local MASTER_WORLD = "MAP_anduril_master"
local MASTER_LEVEL = "/Game/levelsMaster/MAP_anduril_master"
local MASTER_OPTIONS = "game=/Script/DCGame.GameModeRaceBase"
local TRACK_LEVEL = "/Game/levels/MAP_anduril_track01"
local LIGHTING_LEVEL = "/Game/levels/MAP_anduril_lighting_DAY"
local STEP_TIMEOUT_TICKS = 160

local ORDER = {
    "open_master",
    "stream_lighting",
    "finish_lighting",
    "stream_track",
    "finish_track",
    "assign_grid_slot",
    "confirm_ready_popup",
    "force_start",
}

local flow_index = 1
local flow_tick = 0
local step_started = false
local step_started_tick = 0
local flow_stopped = false

local function log(message)
    print(string.format("%s %s\n", TAG, message))
end

local function valid(object)
    return object and object:IsValid()
end

local function first_valid(class_name)
    local objects = FindAllOf(class_name)
    if not objects then return nil end
    for _, object in ipairs(objects) do
        if valid(object) then
            local full_name = object:GetFullName()
            if not string.find(full_name, "Default__", 1, true) then return object end
        end
    end
    return nil
end

local function find_named(class_name, fragment)
    local objects = FindAllOf(class_name)
    if not objects then return nil end
    for _, object in ipairs(objects) do
        if valid(object) and string.find(object:GetFullName(), fragment, 1, true) then
            return object
        end
    end
    return nil
end

local function master_world()
    return find_named("World", MASTER_WORLD)
end

local function current_game_world()
    local worlds = FindAllOf("World")
    if not worlds then return nil end
    for _, world in ipairs(worlds) do
        if valid(world) and string.find(world:GetFullName(), "/Game/", 1, true) then
            return world
        end
    end
    return nil
end

local function controller()
    return first_valid("DronePlayerController")
end

local function stream_level(level_name, callback_name, uuid)
    local manager = first_valid("MapLoadingManager")
    local world = master_world()
    if not valid(manager) or not valid(world) then return false end
    local ok, result = pcall(function()
        local gameplay_statics = StaticFindObject("/Script/Engine.Default__GameplayStatics")
        gameplay_statics:LoadStreamLevel(
            world,
            FName(level_name),
            true,
            true,
            {}
        )
        return true
    end)
    log(string.format("stream level=%s callback=%s ok=%s result=%s",
        level_name, callback_name, tostring(ok), tostring(result)))
    return ok and result
end

local function track_ready()
    local game_state = first_valid("GameStateFlightBase")
    if not valid(game_state) then return false end
    local ok, ready = pcall(function()
        if not game_state:IsTrackLevelLoaded() then return false end
        local track = game_state:GetCurrentTrack()
        if not valid(track) or not track:IsTrackValid() then return false end
        return valid(track:GetStartingGrid())
    end)
    return ok and ready
end

local STEPS = {}

STEPS.open_master = {
    ready = function() return valid(current_game_world()) end,
    run = function()
        if valid(master_world()) then return true end
        local gameplay_statics = StaticFindObject("/Script/Engine.Default__GameplayStatics")
        gameplay_statics:OpenLevel(current_game_world(), FName(MASTER_LEVEL), true, MASTER_OPTIONS)
        return true
    end,
    done = function() return valid(master_world()) and valid(controller()) end,
}

STEPS.stream_lighting = {
    ready = function() return valid(master_world()) and valid(controller()) end,
    run = function()
        return stream_level(LIGHTING_LEVEL, "OnLightingLevelLoaded", 1)
    end,
    done = function() return valid(find_named("Level", "MAP_anduril_lighting_DAY")) end,
}

STEPS.finish_lighting = {
    ready = function()
        return valid(find_named("Level", "MAP_anduril_lighting_DAY"))
            and valid(first_valid("MapLoadingManager"))
    end,
    run = function()
        first_valid("MapLoadingManager"):OnLightingLevelLoaded()
        return true
    end,
    done = function() return true end,
}

STEPS.stream_track = {
    ready = function() return valid(master_world()) and valid(controller()) end,
    run = function()
        return stream_level(TRACK_LEVEL, "OnTrackLevelLoaded", 0)
    end,
    done = function() return valid(find_named("Level", "MAP_anduril_track01")) end,
}

STEPS.finish_track = {
    ready = function()
        return valid(find_named("Level", "MAP_anduril_track01"))
            and valid(first_valid("MapLoadingManager"))
    end,
    run = function()
        first_valid("MapLoadingManager"):OnTrackLevelLoaded()
        return true
    end,
    done = track_ready,
}

STEPS.assign_grid_slot = {
    ready = function()
        return track_ready() and valid(first_valid("GameStateRaceBase"))
            and valid(first_valid("PlayerStateRaceBase"))
    end,
    run = function()
        local game_state = first_valid("GameStateRaceBase")
        local player_state = first_valid("PlayerStateRaceBase")
        if player_state.StartingGridSlot < 0 then
            local slot = game_state:FindAvailableStartingGridSlot()
            log("grid candidate=" .. tostring(slot))
            if slot >= 0 then player_state.StartingGridSlot = slot end
        end
        return true
    end,
    done = function()
        local player_state = first_valid("PlayerStateRaceBase")
        return valid(player_state) and player_state.StartingGridSlot >= 0
    end,
}

STEPS.confirm_ready_popup = {
    ready = function()
        local game_mode = first_valid("GameModeRaceBase")
        return valid(game_mode) and valid(game_mode.OpponentsPopup)
    end,
    run = function()
        first_valid("GameModeRaceBase").OpponentsPopup:ConfirmPopup()
        return true
    end,
    done = function() return true end,
}

STEPS.force_start = {
    ready = function() return track_ready() and valid(controller()) end,
    run = function()
        local game_state = first_valid("GameStateRaceBase")
        if valid(game_state) then
            local ok, started = pcall(function() return game_state:HasRaceStarted() end)
            if ok and started then return true end
        end
        controller():ServerForceStartRaceWithoutRaceVerification()
        return true
    end,
    done = function()
        local game_state = first_valid("GameStateRaceBase")
        if not valid(game_state) then return false end
        local ok, started = pcall(function() return game_state:HasRaceStarted() end)
        return ok and started
    end,
}

local function run_flow()
    if flow_stopped then return end
    flow_tick = flow_tick + 1
    ExecuteInGameThread(function()
        local step_name = ORDER[flow_index]
        if not step_name then
            log("complete physics_ready=true")
            flow_stopped = true
            return
        end
        local step = STEPS[step_name]
        if not step_started then
            local ready_ok, ready = pcall(step.ready)
            if ready_ok and ready then
                local run_ok, run_result = pcall(step.run)
                log(string.format("step=%s run_ok=%s result=%s",
                    step_name, tostring(run_ok), tostring(run_result)))
                if run_ok and run_result ~= false then
                    step_started = true
                    step_started_tick = flow_tick
                else
                    log("failed step=" .. step_name .. " phase=run")
                    flow_stopped = true
                end
            elseif flow_tick % 20 == 0 then
                log(string.format("step=%s waiting ready_ok=%s", step_name, tostring(ready_ok)))
            end
        else
            local done_ok, done = pcall(step.done)
            if done_ok and done then
                log("step=" .. step_name .. " done=true")
                flow_index = flow_index + 1
                step_started = false
            elseif flow_tick - step_started_tick > STEP_TIMEOUT_TICKS then
                log(string.format("failed step=%s done_ok=%s", step_name, tostring(done_ok)))
                flow_stopped = true
            end
        end
    end)
    if not flow_stopped then ExecuteWithDelay(250, run_flow) end
end

log("loaded flow=" .. table.concat(ORDER, ","))
ExecuteWithDelay(500, run_flow)
