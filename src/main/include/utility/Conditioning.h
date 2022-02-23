#pragma once

namespace cwtech
{
    class UniformConditioning
    {
    private:
        double m_deadband = 0.1;
        double m_power    = 1.0;
        double m_min      = 0.0;
        double m_max      = 1.0;

        double m_range    = 1.0;
        double m_mult     = 0.0;

        void Precompute ();
    
    public:
        UniformConditioning();
        void   SetDeadband (double db);
        void   SetExponent (double expo);
        void   SetRange    (double min, double max);

        double Condition   (double x);
    };

    class NonUniformConditioning {
    public:
        UniformConditioning Negative;
        UniformConditioning Positive;

        double Condition (double x);
    };

    double JSCPower (double base, double power);
    inline double JSCSgn (double num);
}