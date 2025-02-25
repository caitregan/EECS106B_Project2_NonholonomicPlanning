
"use strict";

let RegisterGui = require('./RegisterGui.js')
let LoadMap = require('./LoadMap.js')
let MoveRobot = require('./MoveRobot.js')
let AddRfidTag = require('./AddRfidTag.js')
let DeleteThermalSource = require('./DeleteThermalSource.js')
let DeleteCO2Source = require('./DeleteCO2Source.js')
let AddSoundSource = require('./AddSoundSource.js')
let DeleteSoundSource = require('./DeleteSoundSource.js')
let AddThermalSource = require('./AddThermalSource.js')
let LoadExternalMap = require('./LoadExternalMap.js')
let DeleteRfidTag = require('./DeleteRfidTag.js')
let AddCO2Source = require('./AddCO2Source.js')

module.exports = {
  RegisterGui: RegisterGui,
  LoadMap: LoadMap,
  MoveRobot: MoveRobot,
  AddRfidTag: AddRfidTag,
  DeleteThermalSource: DeleteThermalSource,
  DeleteCO2Source: DeleteCO2Source,
  AddSoundSource: AddSoundSource,
  DeleteSoundSource: DeleteSoundSource,
  AddThermalSource: AddThermalSource,
  LoadExternalMap: LoadExternalMap,
  DeleteRfidTag: DeleteRfidTag,
  AddCO2Source: AddCO2Source,
};
