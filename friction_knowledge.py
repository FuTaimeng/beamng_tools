# /*******************************************************************************

# The documentation can be found at

# https://documentation.beamng.com/modding/levels/physics_materials/

# *******************************************************************************/

material_info = {
    
  "DEFAULT": {
    "index"                      : 0,
    "annotation"                 : (0, 0, 0),

    "staticFrictionCoefficient"  : 0.98,
    "slidingFrictionCoefficient" : 0.70,
    "hydrodynamicFriction"       : 0,
    "stribeckVelocity"           : 4.5,
    "strength"                   : 1,
    "roughnessCoefficient"       : 0,

    "defaultDepth"               : 0,
    "collisiontype"              : "ASPHALT",
    "skidMarks"                  : True,
    "aliases"                    : ["groundmodel_asphalt1", "grid", "concrete", "concrete2"]
  },

  "ASPHALT": {
    "index"                      : 1,
    "annotation"                 : (128, 128, 128),

    "staticFrictionCoefficient"  : 0.98,
    "slidingFrictionCoefficient" : 0.70,
    "hydrodynamicFriction"       : 0,
    "stribeckVelocity"           : 4.5,
    "strength"                   : 1,
    "roughnessCoefficient"       : 0,

    "defaultDepth"               : 0,
    "collisiontype"              : "ASPHALT",
    "skidMarks"                  : True,
    "aliases"                    : ["groundmodel_asphalt1", "grid", "concrete", "concrete2"]
  },

  # "ASPHALT_WET": {
  #   "staticFrictionCoefficient"  : 0.92,
  #   "slidingFrictionCoefficient" : 0.55,
  #   "hydrodynamicFriction"       : 0,
  #   "stribeckVelocity"           : 5,
  #   "strength"                   : 1,
  #   "roughnessCoefficient"       : 0.2,

  #   "defaultDepth"               : 0,
  #   "collisiontype"              : "ASPHALT_WET",
  #   "skidMarks"                  : False,
  #   "aliases"                    : ["asphalt_wet2", "asphalt_wet3"]
  # },

  # "ASPHALT_OLD": {
  #   "staticFrictionCoefficient"  : 0.96,
  #   "slidingFrictionCoefficient" : 0.67,
  #   "hydrodynamicFriction"       : 0,
  #   "stribeckVelocity"           : 4.5,
  #   "strength"                   : 1,
  #   "roughnessCoefficient"       : 0,

  #   "defaultDepth"               : 0,
  #   "collisiontype"              : "ASPHALT",
  #   "skidMarks"                  : True,
  #   "aliases"                    : ["groundmodel_asphalt_old"]
  # },

  # "ASPHALT_PREPPED": {
  #   "staticFrictionCoefficient"  : 1.5,
  #   "slidingFrictionCoefficient" : 1.2,
  #   "hydrodynamicFriction"       : 0,
  #   "stribeckVelocity"           : 6,
  #   "strength"                   : 1,
  #   "roughnessCoefficient"       : 0,

  #   "defaultDepth"               : 0,
  #   "collisiontype"              : "ASPHALT",
  #   "skidMarks"                  : True
  #   # "aliases"                  : [""]
  # },

  # speed bump
  "RUMBLE_STRIP": {
    "index"                      : 2,
    "annotation"                 : (196, 64, 64),

    "staticFrictionCoefficient"  : 0.96,
    "slidingFrictionCoefficient" : 0.67,
    "hydrodynamicFriction"       : 0,
    "stribeckVelocity"           : 4.5,
    "strength"                   : 1,
    "roughnessCoefficient"       : 0,

    "defaultDepth"               : 0,
    "collisiontype"              : "RUMBLE_STRIP",
    "skidMarks"                  : False
    # "aliases"                    : [""]
  },

  "ROCK":{
    "index"                      : 3,
    "annotation"                 : (96, 96, 96),

    "staticFrictionCoefficient"  : 0.93,
    "slidingFrictionCoefficient" : 0.62,
    "hydrodynamicFriction"       : 0,
    "stribeckVelocity"           : 4,
    "strength"                   : 1,
    "roughnessCoefficient"       : 0.15,

    "defaultDepth"               : 0,
    "collisiontype"              : "ROCK",
    "skidMarks"                  : False,
    "aliases"                    : ["rock_cliff", "rocks_large"]
  },

  "COBBLESTONE":{
    "index"                      : 4,
    "annotation"                 : (32, 64, 196),

    "staticFrictionCoefficient"  : 0.95,
    "slidingFrictionCoefficient" : 0.65,
    "hydrodynamicFriction"       : 0,
    "stribeckVelocity"           : 5,
    "strength"                   : 1,
    "roughnessCoefficient"       : 0.05,

    "defaultDepth"               : 0,
    "collisiontype"              : "COBBLESTONE",
    "skidMarks"                  : True,
    "aliases"                    : [""]
  },

  # DIRT (Clay like dirt, for dirt roads or terrain)
  "DIRT":{
    "index"                      : 5,
    "annotation"                 : (128, 255, 128), # nature
    "appearance_h_range"         : (0, 60//2),
    "appearance_s_range"         : (25, 255),
    "new_annotation"             : (180, 160, 140),

    "staticFrictionCoefficient"  : 0.68,
    "slidingFrictionCoefficient" : 0.76,
    "hydrodynamicFriction"       : 0.009,
    "stribeckVelocity"           : 8,
    "strength"                   : 1,
    "roughnessCoefficient"       : 0.42,

    "fluidDensity"               : 14000,
    "flowConsistencyIndex"       : 2100,
    "flowBehaviorIndex"          : 0.75,
    "dragAnisotropy"             : 0.5,
    "shearStrength"              : 2500,
    "defaultDepth"               : 0,
    "collisiontype"              : "DIRT",
    "skidMarks"                  : False,
    "aliases"                    : ["dirt_grass", "derby_dirt"]
  },

  # "DIRT_DUSTY":{//DIRT_DUSTY (Dry, hard dusty dirt surface, for dusty dirt roads)
  #   "staticFrictionCoefficient"  : 0.68,
  #   "slidingFrictionCoefficient" : 0.78,
  #   "hydrodynamicFriction"       : 0.0085,
  #   "stribeckVelocity"           : 3.5,
  #   "strength"                   : 1,
  #   "roughnessCoefficient"       : 0.41,

  #   "fluidDensity"               : 12000,
  #   "flowConsistencyIndex"       : 2000,
  #   "flowBehaviorIndex"          : 0.75,
  #   "dragAnisotropy"             : 0.5,
  #   "shearStrength"              : 4200,
  #   "defaultDepth"               : 0,
  #   "collisiontype"              : "DIRT_DUSTY",
  #   "skidMarks"                  : False,
  #   "aliases"                    : ["rockydirt", "dirt_rocky", "dirt_rocky_large"]
  # },

  # "DIRT_DUSTY_LOOSE":{//DIRT_DUSTY_LOOSE (Dry, loose dusty surface, for terrain)
  #   "staticFrictionCoefficient"  : 0.65,
  #   "slidingFrictionCoefficient" : 0.65,
  #   "hydrodynamicFriction"       : 0.012,
  #   "stribeckVelocity"           : 1,
  #   "strength"                   : 1,
  #   "roughnessCoefficient"       : 0.43,

  #   "fluidDensity"               : 10000,
  #   "flowConsistencyIndex"       : 1800,
  #   "flowBehaviorIndex"          : 0.75,
  #   "dragAnisotropy"             : 0.5,
  #   "shearStrength"              : 3500,
  #   "defaultDepth"               : 0.02,
  #   "collisiontype"              : "DIRT_DUSTY",
  #   "skidMarks"                  : False,
  #   "aliases"                    : ["dirt_loose_dusty", "dirt_sandy"]
  # },

  # "GRAVEL" : {
  #   "index"                      : 6,
  #   "annotation"                 : (128, 255, 128), # nature

  #   "staticFrictionCoefficient"  : 0.68,
  #   "slidingFrictionCoefficient" : 0.76,
  #   "hydrodynamicFriction"       : 0.0095,
  #   "stribeckVelocity"           : 8,
  #   "strength"                   : 1,
  #   "roughnessCoefficient"       : 0.41,

  #   "fluidDensity"               : 16000,
  #   "flowConsistencyIndex"       : 2500,
  #   "flowBehaviorIndex"          : 0.75,
  #   "dragAnisotropy"             : 0.5,
  #   "shearStrength"              : 4000,
  #   "defaultDepth"               : 0.0,
  #   "collisiontype"              : "GRAVEL",
  #   "skidMarks"                  : False,
  #   "aliases"                    : ["dirt_loose"]
  # },

  # "GRAVEL_WET" : {
  #   "staticFrictionCoefficient"  : 0.62,
  #   "slidingFrictionCoefficient" : 0.60,
  #   "hydrodynamicFriction"       : 0.0080,
  #   "stribeckVelocity"           : 6,
  #   "strength"                   : 1,
  #   "roughnessCoefficient"       : 0.38,

  #   "fluidDensity"               : 16000,
  #   "flowConsistencyIndex"       : 3000,
  #   "flowBehaviorIndex"          : 0.75,
  #   "dragAnisotropy"             : 0.5,
  #   "shearStrength"              : 5000,
  #   "defaultDepth"               : 0,
  #   "collisiontype"              : "GRAVEL",
  #   "skidMarks"                  : False,
  #   "aliases"                    : ["gravel_wet, gravel_riverbed"]
  # },

  "GRASS":{
    "index"                      : 7,
    "annotation"                 : (64, 128, 64),

    "staticFrictionCoefficient"  : 0.61,
    "slidingFrictionCoefficient" : 0.65,
    "hydrodynamicFriction"       : 0.005,
    "stribeckVelocity"           : 4,
    "strength"                   : 1,
    "roughnessCoefficient"       : 0.43,

    "fluidDensity"               : 8000,
    "flowConsistencyIndex"       : 1500,
    "flowBehaviorIndex"          : 0.7,
    "dragAnisotropy"             : 0,
    "shearStrength"              : 4000,
    "defaultDepth"               : 0.05,
    "collisiontype"              : "GRASS",
    "skidMarks"                  : False,
    "aliases"                    : ["grass", "grass2", "grass3", "grass4", "forest", "forest_floor"]
  },

  "GRASS2":{
    "index"                      : 7,
    "annotation"                 : (128, 255, 128), # nature
    "appearance_h_range"         : (60//2, 180//2),
    "appearance_s_range"         : (25, 255),
    "new_annotation"             : (64, 128, 64),

    "staticFrictionCoefficient"  : 0.61,
    "slidingFrictionCoefficient" : 0.65,
    "hydrodynamicFriction"       : 0.005,
    "stribeckVelocity"           : 4,
    "strength"                   : 1,
    "roughnessCoefficient"       : 0.43,

    "fluidDensity"               : 8000,
    "flowConsistencyIndex"       : 1500,
    "flowBehaviorIndex"          : 0.7,
    "dragAnisotropy"             : 0,
    "shearStrength"              : 4000,
    "defaultDepth"               : 0.05,
    "collisiontype"              : "GRASS",
    "skidMarks"                  : False,
    "aliases"                    : ["grass", "grass2", "grass3", "grass4", "forest", "forest_floor"]
  },

  "MUD":{
    "index"                      : 8,
    "annotation"                 : (128, 96, 32),

    "staticFrictionCoefficient"  : 0.55,
    "slidingFrictionCoefficient" : 0.55,
    "hydrodynamicFriction"       : 0.01,
    "stribeckVelocity"           : 6,
    "strength"                   : 1,
    "roughnessCoefficient"       : 0.5,

    "fluidDensity"               : 7000,
    "flowConsistencyIndex"       : 2000,
    "flowBehaviorIndex"          : 0.5,
    "dragAnisotropy"             : 0.75,
    "shearStrength"              : 4000,
    "defaultDepth"               : 0.15,
    "collisiontype"              : "MUD",
    "skidMarks"                  : False
  },

  "SAND":{
    "index"                      : 9,
    "annotation"                 : (255, 196, 32),

    "staticFrictionCoefficient"  : 0.6,
    "slidingFrictionCoefficient" : 0.6,
    "hydrodynamicFriction"       : 0.02,
    "stribeckVelocity"           : 6,
    "strength"                   : 1,
    "roughnessCoefficient"       : 0.5,

    "fluidDensity"               : 25000,
    "flowConsistencyIndex"       : 5000,
    "flowBehaviorIndex"          : 0.25,
    "dragAnisotropy"             : 0.5,
    "shearStrength"              : 12000,
    "defaultDepth"               : 0.1,
    "collisiontype"              : "SAND",
    "skidMarks"                  : False,
    "aliases"                    : ["beachsand", "sandtrap"]
  },

  "ICE":{
    "index"                      : 10,
    "annotation"                 : (128, 255, 128), # nature
    "appearance_h_range"         : (0, 180),
    "appearance_s_range"         : (0, 25),
    "new_annotation"             : (240, 240, 240),

    "staticFrictionCoefficient"  : 0.4,
    "slidingFrictionCoefficient" : 0.2,
    "hydrodynamicFriction"       : 0,
    "stribeckVelocity"           : 0.5,
    "strength"                   : 1,
    "roughnessCoefficient"       : 0.1,

    "defaultDepth"               : 0,
    "collisiontype"              : "ICE",
    "skidMarks"                  : False
  },

  # "SNOW":{
  #   "staticFrictionCoefficient"  : 0.65,
  #   "slidingFrictionCoefficient" : 0.4,
  #   "hydrodynamicFriction"       : 0.01,
  #   "stribeckVelocity"           : 3,
  #   "strength"                   : 1,
  #   "roughnessCoefficient"       : 0.3,

  #   "fluidDensity"               : 481,
  #   "flowConsistencyIndex"       : 300,
  #   "flowBehaviorIndex"          : 0.8,
  #   "dragAnisotropy"             : 0,
  #   "shearStrength"              : 0,
  #   "defaultDepth"               : 0,
  #   "collisiontype"              : "SNOW",
  #   "skidMarks"                  : False
  # },

}

