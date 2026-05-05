# SPDX-License-Identifier: ISC
# Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

import inspect
import os

import pytest

# Spice Unit Test
# Purpose: Validate SpiceInterface ephemeris output against the JPL Horizons
#          database for several planets across multiple times of year.

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
from xmera import __path__
bsk_path = __path__[0]


import datetime
from xmera.utilities import spice_utilities
from xmera.utilities import unitTestSupport
from xmera.utilities import SimulationBaseClass
import numpy
from xmera.simulation import spiceInterface
from xmera.utilities import macros
import matplotlib.pyplot as plt


# Stores per-parameterized-run errors so we can plot them all together at module teardown.
class DataStore:
    def __init__(self):
        self.date = []
        self.mars_pos_err = []
        self.earth_pos_err = []
        self.sun_pos_err = []

    def plot_data(self):
        fig1 = plt.figure(1)
        rect = fig1.patch
        rect.set_facecolor("white")

        plt.xticks(numpy.arange(len(self.date)), self.date)
        plt.plot(self.mars_pos_err, "r", label="Mars")
        plt.plot(self.earth_pos_err, "bs", label="Earth")
        plt.plot(self.sun_pos_err, "yo", label="Sun")

        plt.rc("font", size=50)
        plt.legend(loc="upper left")
        fig1.autofmt_xdate()
        plt.xlabel("Date of test")
        plt.ylabel("Position Error [m]")
        plt.show()

    def give_data(self):
        plt.figure(1)
        plt.close(1)
        fig1 = plt.figure(1, figsize=(7, 5), dpi=80, facecolor="w", edgecolor="k")

        plt.xticks(numpy.arange(len(self.date)), self.date)
        plt.plot(self.mars_pos_err, "r", label="Mars")
        plt.plot(self.earth_pos_err, "b", label="Earth")
        plt.plot(self.sun_pos_err, "y", label="Sun")

        plt.legend(loc="upper left")
        fig1.autofmt_xdate()
        plt.xlabel("Date of test")
        plt.ylabel("Position Error [m]")
        return plt


@pytest.fixture(scope="module")
def test_plotting_fixture(show_plots):
    data_store = DataStore()
    yield data_store

    plt = data_store.give_data()
    unitTestSupport.writeFigureLaTeX("EphemMars", "Ephemeris Error on Mars", plt,
                                     "height=0.7\\textwidth, keepaspectratio", path)
    plt.ylim(0, 2E-2)
    unitTestSupport.writeFigureLaTeX("EphemEarth", "Ephemeris Error on Earth", plt,
                                     "height=0.7\\textwidth, keepaspectratio", path)
    plt.ylim(0, 5E-6)
    unitTestSupport.writeFigureLaTeX("EphemSun", "Ephemeris Error on Sun", plt,
                                     "height=0.7\\textwidth, keepaspectratio", path)
    if show_plots:
        data_store.plot_data()


@pytest.mark.parametrize("date_spice, date_plot, mars_truth_pos, earth_truth_pos, sun_truth_pos, use_msg", [
    ("2015 February 10, 00:00:00.0 TDB", "02/10/15", [2.049283795042291E+08, 4.654550957513031E+07, 1.580778617009296E+07], [-1.137790671899544E+08, 8.569008401822130E+07, 3.712507705247846E+07], [4.480338216752146E+05, -7.947764237588293E+04, -5.745748832696378E+04], False),
    ("2015 February 20, 00:00:00.0 TDB", "02/20/15", [1.997780190793433E+08, 6.636509140613769E+07, 2.503734390917690E+07], [-1.286636391244711E+08, 6.611156946652703E+07, 2.863736192793162E+07], [4.535258225415821E+05, -7.180260790174162E+04, -5.428151563919459E+04], False),
    ("2015 April 10, 00:00:00.0 TDB", "04/10/15", [1.468463828343225E+08, 1.502909016357645E+08, 6.495986693265321E+07], [-1.406808744055921E+08, -4.614996219219251E+07, -2.003047222725225E+07], [4.786772771370058E+05, -3.283487082146838E+04, -3.809690999538498E+04], False),
    ("2015 April 20, 00:00:00.0 TDB", "04/20/15", [1.313859463489376E+08, 1.637422864185919E+08, 7.154683454681394E+07], [-1.304372112275012E+08, -6.769916175223185E+07, -2.937179538983412E+07], [4.834188130324496E+05, -2.461799304268214E+04, -3.467083541217462E+04], False),
    ("2015 June 10, 00:00:00.0 TDB", "06/10/15", [3.777078276008123E+07, 2.080046702252371E+08, 9.437503998071109E+07], [-2.946784585692780E+07, -1.365779215199542E+08, -5.923299652516938E+07], [5.052070933145228E+05, 1.837038638578682E+04, -1.665575509449521E+04], False),
    ("2015 June 20, 00:00:00.0 TDB", "06/20/15", [1.778745850655047E+07, 2.116712756672253E+08, 9.659608128589308E+07], [-4.338053580692466E+06, -1.393743714818930E+08, -6.044500073550620E+07], [5.090137386469169E+05, 2.694272566092012E+04, -1.304862690440150E+04], False),
    ("2015 August 10, 00:00:00.0 TDB", "08/10/15", [-8.302866300591002E+07, 2.053384243312354E+08, 9.641192179982041E+07], [1.111973130587749E+08, -9.507060674145325E+07, -4.123957889640842E+07], [5.263951240980130E+05, 7.113788303899391E+04, 5.601415938789949E+03], False),
    ("2015 August 20, 00:00:00.0 TDB", "08/20/15", [-1.015602120938274E+08, 1.994877113707506E+08, 9.422840996376510E+07], [1.267319535735967E+08, -7.664279872369213E+07, -3.325170492059625E+07], [5.294368386862668E+05, 7.989635630604146E+04, 9.307380417148834E+03], False),
    ("2015 October 10, 00:00:00.0 TDB", "10/10/15", [-1.828944464171771E+08, 1.498117608528323E+08, 7.363786404040858E+07], [1.440636517249971E+08, 3.827074461651713E+07, 1.656440704503509E+07], [5.433268959250135E+05, 1.251717566539142E+05, 2.849999378507032E+04], False),
    ("2015 October 20, 00:00:00.0 TDB", "10/20/15", [-1.955685835661002E+08, 1.367530082668284E+08, 6.799006120628108E+07], [1.343849818314204E+08, 6.019977403127116E+07, 2.607024615553362E+07], [5.457339294611338E+05, 1.342148834831663E+05, 3.234185290692726E+04], False),
    ("2015 December 10, 00:00:00.0 TDB", "12/10/15", [-2.392022492203017E+08, 5.847873056287902E+07, 3.326431285934674E+07], [3.277145427626925E+07, 1.320956053465003E+08, 5.723804900679157E+07], [5.559607335969181E+05, 1.813263443761486E+05, 5.242991145066972E+04], False),
    ("2015 December 20, 00:00:00.0 TDB", "12/20/15", [-2.432532635321551E+08, 4.156075241419497E+07, 2.561357000250468E+07], [6.866134677340530E+06, 1.351080593806486E+08, 5.854460725992832E+07], [5.575179227151767E+05, 1.907337298309725E+05, 5.645553733620559E+04], False),
    ("2016 February 10, 00:00:00.0 TDB", "02/10/16", [-2.385499316808322E+08, -4.818711325555268E+07, -1.567983931044450E+07], [-1.132504603146183E+08, 8.647183658089657E+07, 3.746046979137553E+07], [5.630027736619673E+05, 2.402590276126245E+05, 7.772804647512530E+04], False),
    ("2016 February 20, 00:00:00.0 TDB", "02/20/16", [-2.326189626865872E+08, -6.493600278878165E+07, -2.352250994262638E+07], [-1.282197228963156E+08, 6.695033443521750E+07, 2.899822684259482E+07], [5.635403728358555E+05, 2.498445036007692E+05, 8.185973180152237E+04], False),
    ("2016 April 10, 00:00:00.0 TDB", "04/10/16", [-1.795928090776869E+08, -1.395102817786797E+08, -5.916067235603344E+07], [-1.399773606371217E+08, -4.749126225182984E+07, -2.061331784067504E+07], [5.639699901756521E+05, 2.977755140828988E+05, 1.025609223621660E+05], False),
    ("2016 April 20, 00:00:00.0 TDB", "04/20/16", [-1.646488222290798E+08, -1.517361128528306E+08, -6.517204439842616E+07], [-1.294310199316289E+08, -6.890085351639988E+07, -2.989515576444890E+07], [5.636348418836893E+05, 3.073681895822543E+05, 1.067117713233756E+05], False),
    ("2016 June 10, 00:00:00.0 TDB", "06/10/16", [-7.085675304355654E+07, -1.933788289169757E+08, -8.680583098365544E+07], [-2.756178030784301E+07, -1.365892814324490E+08, -5.923888961370525E+07], [5.597493293268962E+05, 3.563312003229167E+05, 1.279448896916322E+05], False),
    ("2016 June 20, 00:00:00.0 TDB", "06/20/16", [-4.994076874358515E+07, -1.967336617729592E+08, -8.890950206156498E+07], [-2.413995783152328E+06, -1.390828611356292E+08, -6.032062925759617E+07], [5.585605124166313E+05, 3.659402953599973E+05, 1.321213212542782E+05], False),
    ("2016 August 10, 00:00:00.0 TDB", "08/10/16", [5.965895856067932E+07, -1.851251207911916E+08, -8.654489416392270E+07], [1.124873641861596E+08, -9.344410235679612E+07, -4.053621796412993E+07], [5.501477436262188E+05, 4.149525682073312E+05, 1.534983234437988E+05], False),
    ("2016 August 20, 00:00:00.0 TDB", "08/20/16", [8.021899957229958E+07, -1.770523551156136E+08, -8.339737954004113E+07], [1.277516713236801E+08, -7.484361731649198E+07, -3.247232896320245E+07], [5.480148786547601E+05, 4.245199573757614E+05, 1.576874582857746E+05], False),
    ("2016 October 10, 00:00:00.0 TDB", "10/10/16", [1.674991232418756E+08, -1.090427183510760E+08, -5.456038490399034E+07], [1.434719792031415E+08, 4.029387436854774E+07, 1.744057129058395E+07], [5.348794087050703E+05, 4.727986075510736E+05, 1.788947974297997E+05], False),
    ("2016 October 20, 00:00:00.0 TDB", "10/20/16", [1.797014954219412E+08, -9.133803506487390E+07, -4.676932170648168E+07], [1.334883617893556E+08, 6.209917895944476E+07, 2.689423661839254E+07], [5.318931290166953E+05, 4.821605725626923E+05, 1.830201949404063E+05], False),
    ("2016 December 10, 00:00:00.0 TDB", "12/10/16", [2.087370835407280E+08, 1.035903131428964E+07, -9.084001492689754E+05], [3.082308903715757E+07, 1.328026978502711E+08, 5.754559376449955E+07], [5.147792796720199E+05, 5.293951386498961E+05, 2.038816823549219E+05], False),
    ("2016 December 20, 00:00:00.0 TDB", "12/20/16", [2.075411186038260E+08, 3.092173198610598E+07, 8.555251204561792E+06], [4.885421269793877E+06, 1.355125217382336E+08, 5.872015978003684E+07], [5.110736843893399E+05, 5.385860060393942E+05, 2.079481168492821E+05], True),
])
def test_unitSpice(test_plotting_fixture, show_plots, date_spice, date_plot,
                   mars_truth_pos, earth_truth_pos, sun_truth_pos, use_msg):
    """Module Unit Test"""
    [test_fail_count, test_message] = unit_spice(test_plotting_fixture, show_plots, date_spice, date_plot,
                                                 mars_truth_pos, earth_truth_pos, sun_truth_pos, use_msg)
    assert test_fail_count < 1, test_message


def unit_spice(test_plotting_fixture, show_plots, date_spice, date_plot,
               mars_truth_pos, earth_truth_pos, sun_truth_pos, use_msg):
    test_fail_count = 0
    test_messages = []

    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"

    total_sim = SimulationBaseClass.SimBaseClass()
    dyn_unit_test_proc = total_sim.CreateNewProcess(unit_process_name)
    dyn_unit_test_proc.addTask(total_sim.CreateNewTask(unit_task_name, macros.sec2nano(0.1)))

    spice_object = spiceInterface.SpiceInterface()
    spice_object.modelTag = "SpiceInterfaceData"
    spice_object.SPICEDataPath = bsk_path + "/supportData/EphemerisData/"
    planet_names = ["earth", "mars barycenter", "sun"]
    spice_object.addPlanetNames(planet_names)
    spice_object.UTCCalInit = date_spice

    if use_msg:
        # Verify custom planet frame names are honored, and that "" falls back to the IAU default.
        planet_frames = ["IAU_" + p for p in planet_names]
        planet_frames[1] = ""
        spice_object.planetFrames = planet_frames

    total_sim.AddModelToTask(unit_task_name, spice_object)

    if use_msg:
        epoch_msg = spice_utilities.timeStringToGregorianUTCMsg(date_spice)
        spice_object.epochInMsg.subscribeTo(epoch_msg)

        # The epoch input message must override any UTCCalInit set on the object.
        spice_object.UTCCalInit = "1990 February 10, 00:00:00.0 TDB"

    spice_object_log = spice_object.logger(["GPSSeconds", "J2000Current", "julianDateCurrent", "GPSWeek"])
    total_sim.AddModelToTask(unit_task_name, spice_object_log)

    total_sim.ConfigureStopTime(int(60.0 * 1E9))
    total_sim.InitializeSimulation()
    total_sim.ExecuteSimulation()

    data_gps_sec = unitTestSupport.addTimeColumn(spice_object_log.times(), spice_object_log.GPSSeconds)
    data_jd = unitTestSupport.addTimeColumn(spice_object_log.times(), spice_object_log.julianDateCurrent)

    year = "".join(("20", date_plot[6:8]))
    date = datetime.datetime(int(year), int(date_plot[0:2]), int(date_plot[3:5]))
    test_plotting_fixture.date = date_plot[0:8]

    # Reference epoch for GPS time
    gps_epoch = datetime.datetime(1980, 1, 6)
    time_diff = date - gps_epoch
    gps_time_seconds = time_diff.days * 86400

    julian_start_date = date.toordinal() + 1721424.5

    # Time delta check: difference between GPS seconds and J2000 seconds should stay constant.
    gps_row = data_gps_sec[0, :]
    init_diff = gps_row[1] - gps_row[0] * 1.0E-9
    i = 1
    allow_tolerance = 1E-6
    while i < data_gps_sec.shape[0]:
        if date.isoweekday() == 7:  # Sundays end a GPS week, GPS seconds wrap to 0
            i += 1
        else:
            gps_row = data_gps_sec[i, :]
            curr_diff = gps_row[1] - gps_row[0] * 1.0E-9
            if abs(curr_diff - init_diff) > allow_tolerance:
                test_fail_count += 1
                test_messages.append(f"FAILED: Time delta check failed with difference of: {curr_diff - init_diff:f} \n")
            i += 1

    # Absolute GPS time check (account for the leap second introduced 2015-06-30).
    if date > datetime.datetime(2015, 6, 30):
        gps_end_time = gps_time_seconds + 17 + 60.0 - 68.184  # 17 GPS skip seconds after 2015-06-30
    else:
        gps_end_time = gps_time_seconds + 16 + 60.0 - 67.184

    gps_week = int(gps_end_time / (86400 * 7))
    gps_second_assumed = gps_end_time - gps_week * 86400 * 7
    gps_sec_diff = abs(gps_row[1] - gps_second_assumed)

    allow_tolerance = 1E-4
    if use_msg:
        allow_tolerance = 2E-2
    if date.isoweekday() != 7 and gps_sec_diff > allow_tolerance:
        test_fail_count += 1
        test_messages.append(f"FAILED: Absolute GPS time check failed with difference of: {gps_sec_diff:f} \n")

    # Absolute Julian date check (same leap-second adjustment as above).
    if date > datetime.datetime(2015, 6, 30):
        jd_end_time = julian_start_date + 0.0006944440 - 68.184 / 86400
    else:
        jd_end_time = julian_start_date + 0.0006944440 - 67.184 / 86400

    jd_end_sim = data_jd[i - 1, 1]
    jd_time_error_allow = 0.1 / (24.0 * 3600.0)
    if abs(jd_end_sim - jd_end_time) > jd_time_error_allow:
        test_fail_count += 1
        test_messages.append(f"FAILED: Absolute Julian Date time check failed with difference of: {abs(jd_end_sim - jd_end_time):f} \n")

    pos_err_tolerance = 1000 if use_msg else 250

    # Mars position
    mars_pos_truth = numpy.array(mars_truth_pos) * 1000.0
    mars_pos_vec = spice_object.planetStateOutMsgs[1].read().PositionVector
    mars_pos_diff = numpy.array(mars_pos_vec) - mars_pos_truth
    mars_err = numpy.linalg.norm(mars_pos_diff)
    test_plotting_fixture.mars_pos_err = mars_err
    if mars_err > pos_err_tolerance:
        test_fail_count += 1
        test_messages.append(f"FAILED: Mars position check failed with difference of: {mars_err:f} \n")

    # Earth position
    earth_pos_truth = numpy.array(earth_truth_pos) * 1000.0
    earth_pos_vec = spice_object.planetStateOutMsgs[0].read().PositionVector
    earth_pos_diff = numpy.array(earth_pos_vec) - earth_pos_truth
    earth_err = numpy.linalg.norm(earth_pos_diff)
    test_plotting_fixture.earth_pos_err = earth_err
    if earth_err > pos_err_tolerance:
        test_fail_count += 1
        test_messages.append(f"FAILED: Earth position check failed with difference of: {earth_err:f} \n")

    # Sun position
    sun_pos_truth = numpy.array(sun_truth_pos) * 1000.0
    sun_pos_vec = spice_object.planetStateOutMsgs[2].read().PositionVector
    sun_pos_diff = numpy.array(sun_pos_vec) - sun_pos_truth
    sun_err = numpy.linalg.norm(sun_pos_diff)
    test_plotting_fixture.sun_pos_err = sun_err
    if sun_err > pos_err_tolerance:
        test_fail_count += 1
        test_messages.append(f"FAILED: Sun position check failed with difference of: {sun_err:f} \n")

    if test_fail_count == 0:
        print(" \n PASSED ")

    return [test_fail_count, "".join(test_messages)]


if __name__ == "__main__":
    test_unitSpice(
        None,  # test_plotting_fixture
        True,  # show_plots
        "2015 February 10, 00:00:00.00 TDB",
        "02/10/15",
        [2.049283795042291E+08, 4.654550957513031E+07, 1.580778617009296E+07],
        [-1.137790671899544E+08, 8.569008401822130E+07, 3.712507705247846E+07],
        [4.480338216752146E+05, -7.947764237588293E+04, -5.745748832696378E+04],
        True,  # use_msg
    )
