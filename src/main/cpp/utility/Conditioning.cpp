#include <utility/Conditioning.h>

#include <cmath>

cwtech::UniformConditioning::UniformConditioning()
{
    Precompute();
}

void cwtech::UniformConditioning::SetDeadband(double db)
{
    db = std::abs(db);
    db = std::fmin(db, 1.0);

    m_deadband = db;

    Precompute();
}

void cwtech::UniformConditioning::SetExponent(double expo)
{
    expo = std::fmax(expo, 1.0);

    m_power = expo;

    Precompute();
}

void cwtech::UniformConditioning::SetRange(double min, double max)
{
    m_min = std::fmin(min, max);
    m_max = std::fmax(min, max);

    Precompute();
}

void cwtech::UniformConditioning::Precompute()
{
    m_mult = 1.0 / (1.0 - m_deadband);
    m_range = m_max - m_min;
}

double cwtech::UniformConditioning::Condition(double x)
{
    double xa = std::abs(x);
    double xs = cwtech::JSCSgn(x);
    if (xa < m_deadband) {
        return 0;
    } else {
        return xs * ((cwtech::JSCPower((xa - m_deadband) * m_mult, m_power) * m_range) + m_min);
    }
}

double cwtech::NonUniformConditioning::Condition (double x) 
{
    if (x < 0) return Negative.Condition(x);
    return Positive.Condition(x);
}

inline double cwtech::JSCSgn (double num)
{
    return (0.0 < num) - (num < 0.0);
}

double cwtech::JSCPower (double base, double power)
{
    long ipart = (long)power;
	double fpart = power - ipart;

	switch (ipart) {
	case 0:
		return base;
	case 1:
		return base * (fpart*base + (1-fpart));
	case 2:
		return base*base * (fpart*base + (1-fpart));
	case 3:
		return base*base*base * (fpart*base + (1-fpart));
	default:
		double result = 1.0;
		while (--ipart) result *= base;
		return result * (fpart*base + (1-fpart));
	}
}