require(["LightstreamerClient","Subscription","StaticGrid"],function(LightstreamerClient,Subscription,StaticGrid) {
    var lsClient = new LightstreamerClient("http://push.lightstreamer.com","ISSLIVE");
    lsClient.connectionOptions.setSlowingEnabled(false);

    // USLAB000018: US Current Local Vertical Local Horizontal (LVLH) Attitude Quaternion Component 0
    // USLAB000019: US Current Local Vertical Local Horizontal (LVLH) Attitude Quaternion Component 1
    // USLAB000020: US Current Local Vertical Local Horizontal (LVLH) Attitude Quaternion Component 2
    // USLAB000021: US Current Local Vertical Local Horizontal (LVLH) Attitude Quaternion Component 3
    // USLAB000022: US Attitude Roll Error (deg)
    // USLAB000023: US Attitude Pitch Error (deg)
    // USLAB000024: US Attitude Yaw Error (deg)
    // USLAB000025: US Inertial Attitude Rate X (deg/s)
    // USLAB000026: US Inertial Attitude Rate Y (deg/s)
    // USLAB000027: US Inertial Attitude Rate Z (deg/s)
    // USLAB000028: US Commanded Attitude Quaternion Component 0
    // USLAB000029: US Commanded Attitude Quaternion Component 1
    // USLAB000030: US Commanded Attitude Quaternion Component 2
    // USLAB000031: US Commanded Attitude Quaternion Component 3   

    // USLAB000032: US Guidance, Navigation and Control (GNC) J2000 Propagated State Vector - X (km)
    // USLAB000033: US Guidance, Navigation and Control (GNC) J2000 Propagated State Vector - Y (km)
    // USLAB000034: US Guidance, Navigation and Control (GNC) J2000 Propagated State Vector - Z (km)
    // USLAB000035: US Guidance, Navigation and Control (GNC) J2000 Propagated State Vector - X (m/s)
    // USLAB000036: US Guidance, Navigation and Control (GNC) J2000 Propagated State Vector - Y (m/s)
    // USLAB000037: US Guidance, Navigation and Control (GNC) J2000 Propagated State Vector - Z (m/s)

    var osv = {r: [0.0, 0.0, 0.0], v: [0.0, 0.0, 0.0], ts: null};
    var kepler = {a : 0};

    function updateKepler(kepler)
    {
        document.getElementById("a_value").innerHTML = kepler.a;
        document.getElementById("e_value").innerHTML = kepler.ecc_norm;
        document.getElementById("i_value").innerHTML = kepler.incl;
        document.getElementById("Omega_value").innerHTML = kepler.Omega;
        document.getElementById("omega_value").innerHTML = kepler.omega;
        document.getElementById("E_value").innerHTML = kepler.E;
        document.getElementById("M_value").innerHTML = kepler.M;
        document.getElementById("f_value").innerHTML = Orbits.computeNaturalAnomaly(kepler.ecc_norm, kepler.E);
        document.getElementById("l_value").innerHTML = kepler.Omega + kepler.omega;
        document.getElementById("L_value").innerHTML = kepler.Omega + kepler.omega + kepler.M;
    }

    function updateKeplerInt(kepler)
    {
        document.getElementById("a_value_int").innerHTML = kepler.a;
        document.getElementById("e_value_int").innerHTML = kepler.ecc_norm;
        document.getElementById("i_value_int").innerHTML = kepler.incl;
        document.getElementById("Omega_value_int").innerHTML = kepler.Omega;
        document.getElementById("omega_value_int").innerHTML = kepler.omega;
        document.getElementById("E_value_int").innerHTML = kepler.E;
        document.getElementById("M_value_int").innerHTML = kepler.M;
        document.getElementById("f_value_int").innerHTML = Orbits.computeNaturalAnomaly(kepler.ecc_norm, kepler.E);
        document.getElementById("l_value_int").innerHTML = kepler.Omega + kepler.omega;
        document.getElementById("L_value_int").innerHTML = kepler.Omega + kepler.omega + kepler.M;
    }

    setInterval(function()
    {
        if (MathUtils.norm(osv.r) == 0)
        {
            return;
        }

        kepler = Orbits.osvToKepler(osv.r, osv.v, osv.ts);
        
        updateKepler(kepler);
        let now = new Date();
        let osvProp = Orbits.propagate(kepler, now);
        let kepler_updated = Orbits.osvToKepler(osvProp.r, osvProp.v, osvProp.ts);
        updateKeplerInt(kepler_updated);

        let julian = TimeConversions.computeJulianTime(now);
        let r_ECEF = Coordinates.J2000ToECEF(osvProp.r, julian.JD, julian.JT);
        let lon = MathUtils.atan2d(r_ECEF[1], r_ECEF[0]);
        let lat = MathUtils.rad2Deg(Math.asin(r_ECEF[2] / MathUtils.norm(r_ECEF)));

        document.getElementById("longitude").innerHTML = lon;
        document.getElementById("latitude").innerHTML = lat;

        document.getElementById("position_x_value_int").innerHTML = osvProp.r[0] * 0.001;
        document.getElementById("position_y_value_int").innerHTML = osvProp.r[1] * 0.001;
        document.getElementById("position_z_value_int").innerHTML = osvProp.r[2] * 0.001;
        document.getElementById("velocity_x_value_int").innerHTML = osvProp.v[0] * 0.001;
        document.getElementById("velocity_y_value_int").innerHTML = osvProp.v[1] * 0.001;
        document.getElementById("velocity_z_value_int").innerHTML = osvProp.v[2] * 0.001;
        document.getElementById("osv_delay_int").innerHTML = now.getTime() - kepler.ts.getTime();
        document.getElementById("osv_timestamp_int").innerHTML = now;
    }, 100);

    var sub = new Subscription("MERGE",["USLAB000032", "USLAB000033", "USLAB000034", 
    "USLAB000035", "USLAB000036", "USLAB000037"], ["TimeStamp","Value"]);

    var timeSub = new Subscription('MERGE', 'TIME_000001', ['TimeStamp','Value','Status.Class','Status.Indicator']);
    lsClient.subscribe(sub);
    lsClient.subscribe(timeSub);
    lsClient.connect();

    sub.addListener(
    {
    onSubscription: function() 
    {
        console.log("Subscribed");
    },
    onUnsubscription: function() 
    {
        console.log("Unsubscribed");
    },
    onItemUpdate: function(update) 
    {
        console.log(update.getItemName() + ": " + update.getValue("Value"))
        //fs.appendFile(update.getItemName()+".txt", update.getValue("TimeStamp") + " " + update.getValue("Value") + " " \n");
        if (update.getItemName() == "USLAB000032")
        {
            var posXField = document.getElementById("position_x_value");
            osv.r[0] = parseFloat(update.getValue("Value")) * 1000.0;
            posXField.innerHTML = update.getValue("Value");
            var timeField = document.getElementById("osv_timestamp");
            osv.ts = TimeConversions.timeStampToDate(update.getValue("TimeStamp"));
            timeField.innerHTML = osv.ts;
        }
        if (update.getItemName() == "USLAB000033")
        {
            var posYField = document.getElementById("position_y_value");
            osv.r[1] = parseFloat(update.getValue("Value")) * 1000.0;
            posYField.innerHTML = update.getValue("Value");
        }
        if (update.getItemName() == "USLAB000034")
        {
            var posZField = document.getElementById("position_z_value");
            osv.r[2] = parseFloat(update.getValue("Value")) * 1000.0;
            posZField.innerHTML = update.getValue("Value");
        }
        if (update.getItemName() == "USLAB000035")
        {
            var velXField = document.getElementById("velocity_x_value");
            osv.v[0] = parseFloat(update.getValue("Value"));
            velXField.innerHTML = update.getValue("Value");
        }
        if (update.getItemName() == "USLAB000036")
        {
            var velYField = document.getElementById("velocity_y_value");
            osv.v[1] = parseFloat(update.getValue("Value"));
            velYField.innerHTML = update.getValue("Value");
        }
        if (update.getItemName() == "USLAB000037")
        {
            var velZField = document.getElementById("velocity_z_value");
            osv.v[2] = parseFloat(update.getValue("Value"));
            velZField.innerHTML = update.getValue("Value");
        }
        //updateOsv(osv.r, osv.v);
    }
    });
});
