#ifndef MAGNETIC_DECLINATION_H
#define MAGNETIC_DECLINATION_H

#pragma once

#include <map>
#include <string>

/**
 * @brief This struct represents a row in the .COF file
 */
struct NOAA_COF_COEFFS {
    NOAA_COF_COEFFS() {
        this->n = 0;    // WMM expansion degree
        this->m = 0;    // WMM expansion order
        this->g = 0;    // g Gauss coefficient
        this->h = 0;    // h Gauss coefficient
        this->gdot = 0; // dg / dt
        this->hdot = 0; // dh / dt
    }

    /**
     * @brief Assignment operator overload
     */
    NOAA_COF_COEFFS &operator=(const NOAA_COF_COEFFS& coeffs) {
        this->n = coeffs.n;
        this->m = coeffs.m;
        this->g = coeffs.g;
        this->h = coeffs.h;
        this->gdot = coeffs.gdot;
        this->hdot = coeffs.hdot;

        return *this;
    }

    int n;
    int m;
    double g;
    double h;
    double gdot;
    double hdot;    
};

class MagneticDeclination
{
public:
    MagneticDeclination();
    ~MagneticDeclination();

    /**
     * @brief loads .COF file from @param filPath
     *
     * @param filePath path to .COF file
     *
     * @throws runtime_error Thrown if error in opening .COF file
     *
     * @return
     */
    void LoadCOF(const std::string& filePath);

    /**
     * @brief Calculate the declination angle given a coordinate
     *        From noaa_71569_DS1.pdf Eqn 19
     *
     * @param lambda geodetic longitude
     * @param phi geodetic latitude
     * @param h ellipsoidal height
     * @param t the current year (YYYY)
     *
     * @throws invalid_argument Thrown if latitude is outside of [-90, 90]
     *
     * @return Declination angle in degree
     */
    double CalculateDeclination(double lambda, double phi, double h, double t);

    /**
     * @brief Calculates scalar magnetic potential of the Earth's main magnetic field
     *        From noaa_71569_DS1.pdf Eqn 4
     *
     * @param lambda geodetic longitude
     * @param phi geodetic latitude
     * @param h ellipsoidal height
     * @param t the current year (YYYY)
     *
     * @return scalar magnetic potential
     */
    double v(double lambda, double phiPrime, double r, double t);

    /**
     * @brief Calculates the Gauss coefficient g^m_n(t) at a given time.
     *        From noaa_71569_DS1.pdf Eqn 9
     *
     * @param n the WMM expansion degree
     * @param m the WMM expansion order
     * @param t the current year (YYYY)
     *
     * @return g coefficient at the year t
     */
    double g(int n, int m, double t);

    /**
     * @brief Calculates the Gauss coefficient h^m_n(t) at a given time.
     *        From noaa_71569_DS1.pdf Eqn 9
     *
     * @param n the WMM expansion degree
     * @param m the WMM expansion order
     * @param t the current year (YYYY)
     *
     * @return h coefficient at the year t
     */
    double h(int n, int m, double t);

    /**
     * @brief The Schmidt semi-normalized associated Legendre function evaluated at u, n, m.
     *        From noaa_71569_DS1.pdf Eqn 5
     *
     * @param u value to be evaluated at
     * @param n the WMM expansion degree
     * @param m the WMM expansion order
     *
     * @return pHat
     */
    double pHat(double u, int n, int m);

    /**
     * @brief The partial derivative Schmidt semi-normalized associated Legendre function
     *        in respect to phiPrime.
     *        From noaa_71569_DS1.pdf Eqn 16
     *
     * @param phiPrime geocentric latitude
     * @param n the WMM expansion degree
     * @param m the WMM expansion order
     *
     * @return dPHatdPhiPrime
     */
    double dPHatdPhiPrime(double phiPrime, int n, int m);

    /**
     * @brief The oscillating factor multiplied by the Legendre associated polynomial at u, n, m.
     *        From noaa_71569_DS1.pdf Eqn 6
     *
     * @param u value to be evaluated at
     * @param n the WMM expansion degree
     * @param m the WMM expansion order
     *
     * @return (-1)^m * LegendrePoly(u, n, m)
     */
    double pLegen(double u, double n, double m);

    /**
     * @brief The Legendre associated polynomial at u, n, m.
     *        From IMU_EQs.pdf Eqn 2
     *
     * @param u value to be evaluated at
     * @param n the WMM expansion degree
     * @param m the WMM expansion order
     *
     * @return LegendrePoly(u, n, m)
     */
    double AssociatedLegendrePolynomial(double u, int n, int m);

    /**
     * @brief Calculates a flattened matrix of Legendre's polynomials at every M, L given x.
     *        Stores the resulting array to @ref m_legendrePolynomialMatrix member field.
     *        See @ref GetMthLthOrderAssociatedLegrandreFunctionDerivatives().
     *        From IMU_EQs.pdf Eqn 1
     *
     * @param M the m-th derivative
     * @param L polynomial degree
     * @param x variable, argument to the polynomial
     * 
     * @return
     */
    void CalculateMthLthOrderAssociatedLegrandreFunctionDerivatives(unsigned M, unsigned L, double x);

    /**
     * @brief An array of size = (L + 1) * (M + 1). Array[cols * l + m] returns the M-th derivative
     *         of the Lth-degree polynomial.
     * 
     * @return m_legendrePolynomialMatrix
     */
    double* GetMthLthOrderAssociatedLegrandreFunctionDerivatives();

    /**
     * @brief Field X' North Component in geocentric coordinate of the geomagnetic field
     *        From noaa_71569_DS1.pdf Eqn 10
     * 
     * @param lambda geodetic longitude
     * @param phiPrime geocentric latitude
     * @param r geocentric altitude
     * @param t the current year (YYYY)
     *
     * @return X' vector component
     */
    double xPrime(double lambda, double phiPrime, double r, double t);

    /**
     * @brief Field Y' East component in geocentric coordinate of the geomagnetic field
     *        From noaa_71569_DS1.pdf Eqn 11
     * 
     * @param lambda geodetic longitude
     * @param phiPrime geocentric latitude
     * @param r geocentric altitude
     * @param t the current year (YYYY)
     *
     * @return Y' vector component
     */
    double yPrime(double lambda, double phiPrime, double r, double t);

    /**
     * @brief Field Z' Down component in geocentric coordinate of the geomagnetic field
     *        From noaa_71569_DS1.pdf Eqn 12
     * 
     * @param lambda geodetic longitude
     * @param phiPrime geocentric latitude
     * @param r geocentric altitude
     * @param t the current year (YYYY)
     *
     * @return Z' vector component
     */
    double zPrime(double lambda, double phiPrime, double r, double t);

    /**
     * @brief Calculating the length from center to (lambda, phi, h) projection onto the horizon
     *        From noaa_71569_DS1.pdf Eqn 7
     * 
     * @param r Radius of curvature of the prime vertical
     * @param h ellipsoidal height
     * @param phi geodetic latitude
     *
     * @return distance from center to the projected point
     */
    double p(double r, double h, double phi);

    /**
     * @brief Calculating the length from center to (lambda, phi, h) projection onto the prime vertical
     *        From noaa_71569_DS1.pdf Eqn 7
     * 
     * @param r Radius of curvature of the prime vertical
     * @param h ellipsoidal height
     * @param phi geodetic latitude
     *
     * @return distance from center to the projected point
     */
    double z(double r, double h, double phi);

    /**
     * @brief gets the g Gauss coefficient given n and m
     * 
     * @param n the WMM expansion degree
     * @param m the WMM expansion order
     *
     * @return g Gauss coefficient
     */
    double GetG(int n, int m);

    /**
     * @brief gets the gDot Gauss coefficient given n and m
     * 
     * @param n the WMM expansion degree
     * @param m the WMM expansion order
     *
     * @return gDot Gauss coefficient
     */
    double GetGDot(int n, int m);

    /**
     * @brief gets the h Gauss coefficient given n and m
     * 
     * @param n the WMM expansion degree
     * @param m the WMM expansion order
     *
     * @return h Gauss coefficient
     */
    double GetH(int n, int m);

    /**
     * @brief gets the hDot Gauss coefficient given n and m
     * 
     * @param n the WMM expansion degree
     * @param m the WMM expansion order
     *
     * @return hDot Gauss coefficient
     */
    double GetHDot(int n, int m);

    /**
     * @brief caller to CalculateMthLthOrderAssociatedLegrandreFunctionDerivatives()
     *
     * @return
     */
    void SetAssociatedPolynomialMatrix(double x);

private:

    /**
     * @brief calculates the binomial coefficient
     * 
     * @param n total objects
     * @param r number of objects chosen from n
     *
     * @return binomial coefficient
     */
    double Binomial(int n, int r);
    
    /**
     * @brief calculates the partial factorial
     * 
     * @param n starting number
     * @param m the number of (result *= --n) operations to be run before breaking
     * 
     * @return n x (n - 1) x ... x (n - m + 1)
     */
    double FallingFactorial(double n, int m);

private:
    static const int N;     // WMM Degree of expansion
    static const double A;  // Semi major axis, meters
    static const double F;  // Flattening
    static const double E;  // Eccentricity

    double *m_legendrePolynomialMatrix = static_cast<double*>(calloc(13 * 13, sizeof(double))); // A matrix containing M derivatives of polynomial order N
    std::map<std::pair<int, int>, NOAA_COF_COEFFS> m_coeffMap;  // (n, m) -> (g, h, gDot, gDot)
    double m_epoch; // YYYY considered Epoch, defined in .COF file
};

#endif