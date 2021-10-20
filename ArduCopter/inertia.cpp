#include "Copter.h"

// read_inertia - read inertia in from accelerometers
void Copter::read_inertia()
{
    // inertial altitude estimates. Use barometer climb rate during high vibrations
    // 惯性高度估计。在高振动期间使用气压计爬升率
    inertial_nav.update(vibration_check.high_vibes);

    // pull position from ahrs
    Location loc;
    ahrs.get_position(loc);
    current_loc.lat = loc.lat;
    current_loc.lng = loc.lng;

    // exit immediately if we do not have an altitude estimate
    // 如果没有高度估计，请立即离开
    if (!inertial_nav.get_filter_status().flags.vert_pos) {
        return;
    }

    // current_loc.alt is alt-above-home, converted from inertial nav's alt-above-ekf-origin
    // current_loc.alt 是在家的上方的高度，由惯性导航的alt-above-ekf-origin转换过来
    const int32_t alt_above_origin_cm = inertial_nav.get_altitude();
    current_loc.set_alt_cm(alt_above_origin_cm, Location::AltFrame::ABOVE_ORIGIN);
    if (!ahrs.home_is_set() || !current_loc.change_alt_frame(Location::AltFrame::ABOVE_HOME)) {
        // if home has not been set yet we treat alt-above-origin as alt-above-home
        // 如果家没有设置，我们将alt-above-origin 作为 alt-above-home
        current_loc.set_alt_cm(alt_above_origin_cm, Location::AltFrame::ABOVE_HOME);
    }
}
