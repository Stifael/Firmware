#include <unit_test/unit_test.h>
#include <float.h>

#include "../../lib/bezier/BezierQuad.hpp"

class BezierQuadTest : public UnitTest
{
public:
	virtual bool run_tests();

private:

	bool _get_states_from_time();


};


bool BezierQuadTest::run_tests()
{
	ut_run_test(_get_states_from_time);

	return (_tests_failed == 0);
}

bool BezierQuadTest::_get_states_from_time()
{
	// symmyetric around 0
	matrix::Vector3f pt0(-0.5f, 0.0f, 0.0f);
	matrix::Vector3f ctrl(0.0f, 0.5f, 0.0f);
	matrix::Vector3f pt1(0.5f, 0.0f, 0.0f);

	// create bezier with default t = [0,1]
	bezier::BezierQuadf bz(pt0, ctrl, pt1);

	matrix::Vector3f pos, vel, acc;
	float precision = 0.00001;

	// states at time = 0
	bz.getStates(pos, vel, acc, 0.0f);

	ut_compare_float("pos[0] not equal pt0[0]", pos(0), pt0(0), precision);
	ut_compare_float("pos[1] not equal pt0[1]", pos(1), pt0(1), precision);
	ut_compare_float("pos[2] not equal pt0[2]", pos(2), pt0(2), precision);

	ut_compare_float("slope not equal 1", vel(0), 1.0f, precision);
	ut_compare_float("slope not equal 1", vel(1), 1.0f, precision);
	ut_compare_float("slope not equal 0", vel(2), 0.0f, precision);

	ut_compare_float("acc not equal 0", acc(0), 0.0f, precision);
	ut_compare_float("acc not equal 1", acc(1), -2.0f, precision);
	ut_compare_float("acc not equal 0", acc(2), 0.0f, precision);

	// states at time = 1
	bz.getStates(pos, vel, acc, 1.0f);

	ut_compare_float("pos[0] not equal pt1[0]", pos(0), pt1(0), precision);
	ut_compare_float("pos[1] not equal pt1[1]", pos(1), pt1(1), precision);
	ut_compare_float("pos[2] not equal pt1[2]", pos(2), pt1(2), precision);

	ut_compare_float("slope not equal 1", vel(0), 1.0f, precision);
	ut_compare_float("slope not equal -1", vel(1), -1.0f, precision);
	ut_compare_float("slope not equal 0", vel(2), 0.0f, precision);

	ut_compare_float("acc not equal 0", acc(0), 0.0f, precision);
	ut_compare_float("acc not equal 1", acc(1), -2.0f, precision);
	ut_compare_float("acc not equal 0", acc(2), 0.0f, precision);

	// states at time = 0.5
	bz.getStates(pos, vel, acc, 0.50f);

	// pos must be equal to ctrl(0) and lower than ctrl(1)
	ut_compare_float("pos[0] not equal ctrl[0]", pos(0), ctrl(0), precision);
	ut_assert_true(pos(1) < ctrl(1));

	ut_compare_float("slope not equal 1", vel(0), 1.0f, precision);
	ut_compare_float("slope not equal -1", vel(1), 0.0f, precision);
	ut_compare_float("slope not equal 0", vel(2), 0.0f, precision);

	ut_compare_float("acc not equal 0", acc(0), 0.0f, precision);
	ut_compare_float("acc not equal -2", acc(1), -2.0f, precision);
	ut_compare_float("acc not equal 0", acc(2), 0.0f, precision);

	// acceleration
	pt0 = matrix::Vector3f(0.0f, 0.0f, 0.0f);
	ctrl = matrix::Vector3f(0.0f, 0.0f, 0.0f);
	pt1 = matrix::Vector3f(1.0f, 0.0f, 0.0f);

	// create bezier with default t = [0,1]
	bz.setBezier(pt0, ctrl, pt1, 1.0f);

	// states at time = 0.0
	bz.getStates(pos, vel, acc, 0.0f);

	ut_compare_float("pos[0] not equal pt0[0]", pos(0), pt0(0), precision);
	ut_compare_float("pos[1] not equal pt0[1]", pos(1), pt0(1), precision);
	ut_compare_float("pos[2] not equal pt0[2]", pos(2), pt0(2), precision);

	ut_compare_float("slope not equal 0", vel(0), 0.0f, precision);
	ut_compare_float("slope not equal 0", vel(1), 0.0f, precision);
	ut_compare_float("slope not equal 0", vel(2), 0.0f, precision);

	ut_compare_float("acc not equal 2", acc(0), 2.0f, precision);
	ut_compare_float("acc not equal 0", acc(1), 0.0f, precision);
	ut_compare_float("acc not equal 0", acc(2), 0.0f, precision);

	// states at time = 1.0
	bz.getStates(pos, vel, acc, 1.0f);

	ut_compare_float("pos[0] not equal pt1[0]", pos(0), pt1(0), precision);
	ut_compare_float("pos[1] not equal pt1[1]", pos(1), pt1(1), precision);
	ut_compare_float("pos[2] not equal pt1[2]", pos(2), pt1(2), precision);

	ut_compare_float("slope not equal 2", vel(0), 2.0f, precision);
	ut_compare_float("slope not equal 0", vel(1), 0.0f, precision);
	ut_compare_float("slope not equal 0", vel(2), 0.0f, precision);

	ut_compare_float("acc not equal 2", acc(0), 2.0f, precision);
	ut_compare_float("acc not equal 0", acc(1), 0.0f, precision);
	ut_compare_float("acc not equal 0", acc(2), 0.0f, precision);

	// states at time = 1.0
	bz.getStates(pos, vel, acc, 0.5f);

	ut_compare_float("slope not equal 1", vel(0), 1.0f, precision);
	ut_compare_float("slope not equal 0", vel(1), 0.0f, precision);
	ut_compare_float("slope not equal 0", vel(2), 0.0f, precision);

	ut_compare_float("acc not equal 2", acc(0), 2.0f, precision);
	ut_compare_float("acc not equal 0", acc(1), 0.0f, precision);
	ut_compare_float("acc not equal 0", acc(2), 0.0f, precision);

	return true;

}

ut_declare_test_c(test_bezierQuad, BezierQuadTest)
