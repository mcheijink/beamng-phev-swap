// Plug-in Hybrid Monitor: UI app of the Plug-in Hybrid Swap mod.
// Reads the phev* values the hybrid module and controller publish in the electrics stream.
// Original code written for this mod. License: MIT, (c) 2026 mcheijink.
angular.module('beamng.apps')
.directive('phevMonitor', [function () {
  return {
    template: `
<div class="bngApp phev-mon" style="width:100%; height:100%;" ng-show="d.present">
  <style>
    .phev-mon { font-family: sans-serif; font-size: 12px; color: #e8eef2; background: rgba(12, 16, 22, 0.82); padding: 8px 10px; box-sizing: border-box; }
    .phev-mon .row { display: flex; align-items: center; gap: 6px; margin-bottom: 6px; }
    .phev-mon .modes { display: flex; gap: 4px; flex: 1; }
    .phev-mon .btn { flex: 1; text-align: center; padding: 4px 0; border-radius: 3px; background: #2a3440; color: #9fb0bf; cursor: pointer; font-weight: bold; letter-spacing: 0.5px; user-select: none; }
    .phev-mon .btn.on { background: #1f7ae0; color: #fff; }
    .phev-mon .btn.boost.on { background: #d7263d; }
    .phev-mon .btn.ev.on { background: #19a974; }
    .phev-mon .btn.charge { flex: 0 0 64px; font-size: 10px; }
    .phev-mon .btn.charge.on { background: #e0a21f; color: #111; }
    .phev-mon .ready { font-size: 10px; font-weight: bold; padding: 3px 6px; border-radius: 3px; background: #2a3440; color: #6b7a88; }
    .phev-mon .ready.on { background: #19a974; color: #fff; }
    .phev-mon .label { color: #8796a5; font-size: 10px; text-transform: uppercase; letter-spacing: 0.6px; }
    .phev-mon .value { font-size: 15px; font-weight: bold; font-variant-numeric: tabular-nums; }
    .phev-mon .unit { color: #8796a5; font-size: 10px; margin-left: 2px; }
    .phev-mon .bar { position: relative; height: 14px; background: #2a3440; border-radius: 3px; overflow: hidden; flex: 1; }
    .phev-mon .bar .fill { position: absolute; left: 0; top: 0; bottom: 0; background: #19a974; }
    .phev-mon .bar .mark { position: absolute; top: -2px; bottom: -2px; width: 2px; background: #e0a21f; }
    .phev-mon .grid { display: grid; grid-template-columns: repeat(4, 1fr); gap: 4px 8px; margin-top: 6px; }
    .phev-mon .cell { background: rgba(255,255,255,0.04); border-radius: 3px; padding: 3px 5px; }
    .phev-mon svg text { font-family: sans-serif; }
  </style>

  <div class="row">
    <div class="modes">
      <div class="btn ev" ng-class="{on: d.mode === 0}" ng-click="cmd('toggleElectricMode')">ELECTRIC</div>
      <div class="btn" ng-class="{on: d.mode === 1}" ng-click="setHybrid()">HYBRID</div>
      <div class="btn boost" ng-class="{on: d.mode === 2}" ng-click="cmd('toggleBoostMode')">BOOST</div>
    </div>
    <div class="btn charge" ng-class="{on: d.charge}" ng-click="cmd('toggleChargeMode')">CHARGE</div>
    <div class="btn charge" style="flex: 0 0 28px;" ng-class="{on: d.bMode}" ng-click="cmd('toggleBMode')" title="B position: stronger regeneration">B</div>
    <div class="ready" ng-class="{on: d.ready}">READY</div>
  </div>

  <div class="row">
    <span class="label" style="width:52px;">Battery</span>
    <div class="bar">
      <div class="fill" ng-style="{width: d.soc + '%', background: d.socColor}"></div>
      <div class="mark" ng-style="{left: d.hold + '%'}" title="hold target"></div>
    </div>
    <span class="value" style="width:48px; text-align:right;">{{ d.soc.toFixed(0) }}<span class="unit">%</span></span>
  </div>
  <div class="row" style="justify-content: space-between; margin-top:-3px;">
    <span class="label">{{ d.usableKWh.toFixed(1) }} / {{ d.usableCapKWh.toFixed(1) }} kWh usable · raw {{ d.socRaw.toFixed(1) }}% ({{ d.winMin.toFixed(1) }}–{{ d.winMax.toFixed(0) }}%)</span>
    <span class="label" ng-style="{color: d.battKW < -0.5 ? '#19a974' : (d.battKW > 0.5 ? '#ffb347' : '#8796a5')}">{{ d.battKW > 0 ? '+' : '' }}{{ d.battKW.toFixed(1) }} kW</span>
  </div>

  <svg viewBox="0 0 400 118" style="width:100%; height:auto; display:block;">
    <defs>
      <marker id="phevArrow" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="5" markerHeight="5" orient="auto-start-reverse">
        <path d="M0,0 L10,5 L0,10 z" fill="#c9d4de"></path>
      </marker>
    </defs>
    <!-- engine -->
    <rect x="4" y="8" width="104" height="58" rx="4" ng-attr-fill="{{ d.iceRunning ? '#5a3a1a' : '#222b35' }}" stroke="#4a5866"></rect>
    <text x="56" y="23" text-anchor="middle" fill="#8796a5" font-size="10">1.4 turbo</text>
    <text x="56" y="40" text-anchor="middle" fill="#e8eef2" font-size="13" font-weight="bold">{{ d.iceStateName }}</text>
    <text x="56" y="56" text-anchor="middle" fill="#c9d4de" font-size="10">{{ d.iceRPM.toFixed(0) }} rpm · {{ d.engKW.toFixed(0) }} kW</text>
    <!-- K0 -->
    <rect x="116" y="22" width="22" height="30" rx="3" fill="#222b35" stroke="#4a5866"></rect>
    <rect x="116" ng-attr-y="{{ 52 - 30 * d.k0 }}" width="22" ng-attr-height="{{ 30 * d.k0 }}" rx="3" fill="#1f7ae0"></rect>
    <text x="127" y="64" text-anchor="middle" fill="#8796a5" font-size="9">K0</text>
    <!-- motor -->
    <rect x="146" y="8" width="104" height="58" rx="4" ng-attr-fill="{{ d.motorKW > 0.5 ? '#123a2c' : (d.motorKW < -0.5 ? '#123040' : '#222b35') }}" stroke="#4a5866"></rect>
    <text x="198" y="23" text-anchor="middle" fill="#8796a5" font-size="10">Motor 85 kW</text>
    <text x="198" y="40" text-anchor="middle" fill="#e8eef2" font-size="13" font-weight="bold">{{ d.motorKW > 0 ? '+' : '' }}{{ d.motorKW.toFixed(0) }} kW</text>
    <text x="198" y="56" text-anchor="middle" fill="#c9d4de" font-size="10">{{ d.motorNm.toFixed(0) }} Nm · {{ d.shaftRPM.toFixed(0) }} rpm</text>
    <!-- DCT -->
    <rect x="258" y="8" width="62" height="58" rx="4" fill="#222b35" stroke="#4a5866"></rect>
    <text x="289" y="23" text-anchor="middle" fill="#8796a5" font-size="10">DCT</text>
    <text x="289" y="42" text-anchor="middle" fill="#e8eef2" font-size="16" font-weight="bold">{{ d.gear }}</text>
    <text x="289" y="57" text-anchor="middle" ng-attr-fill="{{ d.launchElectric ? '#19a974' : '#ffb347' }}" font-size="9">{{ d.launchElectric ? 'electric' : 'engine' }}</text>
    <!-- wheels -->
    <rect x="328" y="8" width="68" height="58" rx="4" fill="#222b35" stroke="#4a5866"></rect>
    <text x="362" y="23" text-anchor="middle" fill="#8796a5" font-size="10">Wheels</text>
    <text x="362" y="41" text-anchor="middle" fill="#e8eef2" font-size="14" font-weight="bold">{{ d.speed.toFixed(0) }}</text>
    <text x="362" y="56" text-anchor="middle" fill="#c9d4de" font-size="9">km/h · {{ d.wheelKW.toFixed(0) }} kW</text>
    <!-- battery -->
    <rect x="146" y="84" width="104" height="28" rx="4" fill="#222b35" stroke="#4a5866"></rect>
    <text x="198" y="102" text-anchor="middle" fill="#e8eef2" font-size="11">HV battery {{ d.grossKWh.toFixed(1) }} kWh</text>
    <!-- flows -->
    <line x1="108" y1="37" x2="116" y2="37" stroke="#ffb347" stroke-width="3" ng-attr-opacity="{{ d.engKW > 0.5 ? 1 : 0.15 }}"></line>
    <line x1="138" y1="37" x2="146" y2="37" stroke="#ffb347" stroke-width="3" ng-attr-opacity="{{ d.engKW > 0.5 ? 1 : 0.15 }}"></line>
    <line x1="250" y1="37" x2="258" y2="37" stroke="#c9d4de" stroke-width="3" ng-attr-opacity="{{ abs(d.sysKW) > 0.5 ? 1 : 0.15 }}"></line>
    <line x1="320" y1="37" x2="328" y2="37" stroke="#c9d4de" stroke-width="3" ng-attr-opacity="{{ abs(d.sysKW) > 0.5 ? 1 : 0.15 }}"></line>
    <line x1="198" y1="84" x2="198" y2="68" ng-attr-stroke="{{ d.motorKW < -0.5 ? '#19a974' : '#4fc3f7' }}" stroke-width="3"
          ng-attr-opacity="{{ abs(d.motorKW) > 0.5 ? 1 : 0.15 }}" marker-end="url(#phevArrow)" ng-attr-transform="{{ d.motorKW < -0.5 ? 'rotate(180 198 76)' : '' }}"></line>
  </svg>

  <div class="grid">
    <div class="cell"><div class="label">Boost</div><span class="value">{{ d.boost.toFixed(1) }}</span><span class="unit">psi</span></div>
    <div class="cell"><div class="label">Engine share</div><span class="value">{{ (d.engineShare * 100).toFixed(0) }}</span><span class="unit">%</span></div>
    <div class="cell"><div class="label">Coolant</div><span class="value">{{ d.coolant.toFixed(0) }}</span><span class="unit">°C</span><span class="unit" ng-show="d.warming"> warm-up</span></div>
    <div class="cell"><div class="label">Fuel</div><span class="value">{{ (d.fuel * 100).toFixed(0) }}</span><span class="unit">%</span></div>
    <div class="cell"><div class="label">Hold</div><span class="value">{{ d.hold.toFixed(0) }}</span><span class="unit">%</span></div>
    <div class="cell"><div class="label">Pedal</div><span class="value">{{ (d.pedal * 100).toFixed(0) }}</span><span class="unit">%</span></div>
    <div class="cell"><div class="label">Engine thr.</div><span class="value">{{ (d.iceThrottle * 100).toFixed(0) }}</span><span class="unit">%</span></div>
    <div class="cell"><div class="label">AVAS</div><span class="value">{{ (d.avas * 100).toFixed(0) }}</span><span class="unit">%</span></div>
    <div class="cell"><div class="label">Brake regen</div><span class="value">{{ (d.regenShare * 100).toFixed(0) }}</span><span class="unit">%</span></div>
    <div class="cell"><div class="label">Brake friction</div><span class="value">{{ (d.frictionBrake * 100).toFixed(0) }}</span><span class="unit">%</span></div>
    <div class="cell"><div class="label">TC torque</div><span class="value" ng-style="{color: d.tcFactor < 0.99 ? '#ffb347' : '#e8eef2'}">{{ (d.tcFactor * 100).toFixed(0) }}</span><span class="unit">%</span></div>
    <div class="cell"><div class="label">Pack temp.</div><span class="value">{{ d.battTemp.toFixed(1) }}</span><span class="unit">°C</span></div>
    <div class="cell"><div class="label">Discharge max</div><span class="value" ng-style="{color: d.dischLim < 89 ? '#ffb347' : '#e8eef2'}">{{ d.dischLim.toFixed(0) }}</span><span class="unit">kW</span></div>
    <div class="cell"><div class="label">Charge max</div><span class="value" ng-style="{color: d.chgLim < 39 ? '#ffb347' : '#e8eef2'}">{{ d.chgLim.toFixed(0) }}</span><span class="unit">kW</span></div>
    <div class="cell"><div class="label">Aggression</div><span class="value">{{ (d.aggr * 100).toFixed(0) }}</span><span class="unit">%</span><span class="unit" ng-show="d.bMode"> B</span></div>
    <div class="cell"><div class="label">Kickdown</div><span class="value" ng-style="{color: d.kickdown ? '#d7263d' : '#e8eef2'}">{{ d.kickdownEnabled ? (d.kickdown ? 'ON' : 'ready') : 'off' }}</span></div>
  </div>
</div>`,
    replace: true,
    restrict: 'EA',
    scope: true,
    controller: ['$scope', function ($scope) {
      const streamsList = ['electrics']
      StreamsManager.add(streamsList)
      $scope.$on('$destroy', function () { StreamsManager.remove(streamsList) })

      const stateNames = ['OFF', 'CRANK', 'SYNC', 'CLOSING', 'RUNNING', 'STOPPING']
      const num = (x, fallback) => (typeof x === 'number' && isFinite(x)) ? x : (fallback || 0)

      $scope.abs = Math.abs
      $scope.d = {
        present: false, mode: 0, ready: false, charge: false, soc: 0, socRaw: 0, hold: 0, socColor: '#19a974',
        usableKWh: 0, usableCapKWh: 0, grossKWh: 0, winMin: 0, winMax: 0, battKW: 0,
        iceStateName: 'OFF', iceRunning: false, iceRPM: 0, engKW: 0, k0: 0,
        motorKW: 0, motorNm: 0, shaftRPM: 0, gear: 'P', launchElectric: true, speed: 0, sysKW: 0, wheelKW: 0,
        boost: 0, engineShare: 1, coolant: 0, warming: false, fuel: 0, pedal: 0, iceThrottle: 0, avas: 0,
        regenShare: 0, frictionBrake: 0, tcFactor: 1, kickdown: false, kickdownEnabled: true,
        battTemp: 0, dischLim: 0, chgLim: 0, aggr: 0, bMode: false
      }

      // send a command to the vehicle's phev controller
      $scope.cmd = function (fn) {
        bngApi.activeObjectLua(`controller.getControllerSafe('phev').${fn}()`)
      }
      $scope.setHybrid = function () {
        bngApi.activeObjectLua(`controller.getControllerSafe('phev').setMode(1)`)
      }

      $scope.$on('streamsUpdate', function (event, streams) {
        const e = streams.electrics
        // the app shows only while the active vehicle has the Boost hybrid (by design): hidden on any other car
        if (!e) { $scope.$evalAsync(function () { $scope.d.present = false }); return }
        $scope.$evalAsync(function () {
          const d = $scope.d
          d.present = typeof e.phevMode === 'number'
          d.mode = num(e.phevMode)
          d.ready = num(e.phevReady) > 0
          d.charge = num(e.phevChargeFunction) > 0
          d.soc = Math.max(0, Math.min(100, num(e.phevSocUsable)))
          d.socRaw = num(e.phevSocRaw)
          d.hold = Math.max(0, Math.min(100, num(e.phevHoldTarget)))
          d.socColor = d.soc < 10 ? '#d7263d' : (d.soc < 25 ? '#e0a21f' : '#19a974')
          d.usableKWh = num(e.phevBatteryUsableKWh)
          d.usableCapKWh = num(e.phevBatteryUsableCapKWh)
          d.grossKWh = num(e.phevBatteryGrossKWh)
          d.winMin = num(e.phevSocWindowMin)
          d.winMax = num(e.phevSocWindowMax)
          d.battKW = num(e.phevBatteryKW)
          const stateCode = Math.round(num(e.phevIceStateCode))
          d.iceStateName = stateNames[stateCode] || 'OFF'
          d.iceRunning = stateCode === 4
          d.iceRPM = Math.max(0, num(e.phevIceRPM))
          d.engKW = num(e.phevEnginePowerKW)
          d.k0 = Math.max(0, Math.min(1, num(e.phevK0Ratio)))
          d.motorKW = num(e.phevMotorPowerKW)
          d.motorNm = num(e.phevMotorTorque)
          d.shaftRPM = Math.max(0, num(e.phevShaftRPM))
          d.gear = (e.gear !== undefined && e.gear !== null) ? String(e.gear) : String(num(e.gearIndex))
          d.launchElectric = num(e.phevLaunchElectric) > 0
          d.speed = Math.abs(num(e.wheelspeed)) * 3.6
          d.sysKW = num(e.phevSystemPowerKW)
          d.wheelKW = num(e.phevWheelPowerKW)
          d.boost = num(e.turboBoost)
          d.engineShare = num(e.phevEngineShare, 1)
          d.coolant = num(e.phevCoolantC)
          d.warming = num(e.phevWarmingUp) > 0
          d.fuel = num(e.fuel)
          d.pedal = num(e.throttle)
          d.iceThrottle = num(e.phevIceThrottle)
          d.avas = num(e.phevAvasVolume)
          d.regenShare = num(e.phevRegenBrakeShare)
          d.frictionBrake = num(e.phevFrictionBrake)
          d.tcFactor = num(e.phevTcFactor, 1)
          d.kickdown = num(e.phevKickdown) > 0
          d.kickdownEnabled = num(e.phevKickdownEnabled, 1) > 0
          d.battTemp = num(e.phevBatteryTempC)
          d.dischLim = num(e.phevBattDischargeLimitKW)
          d.chgLim = num(e.phevBattChargeLimitKW)
          d.aggr = num(e.phevAggression)
          d.bMode = num(e.phevBMode) > 0
        })
      })
    }]
  }
}])
