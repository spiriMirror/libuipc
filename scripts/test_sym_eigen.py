from SymEigen import *

if __name__ == '__main__':
    X = Eigen.Vector('X', 6)
    
    k = Eigen.Scalar('k')
    L0 = Eigen.Scalar('L0')

    X_l = Matrix(X[0:3])
    X_r = Matrix(X[3:6])
    d = X_l - X_r 

    E = k * (sqrt(d.T * d) - L0) / 2
    G = VecDiff(E, X)
    H = VecDiff(G, X)
    
    Gen = EigenFunctionGenerator()
    Gen.DisableLatexComment()
    Gen.MacroBeforeFunction("MUDA_GENERIC")
    Closure = Gen.Closure(k, L0, X)
    print(Closure('SpringEnergy', E, 'E'))
    print(Closure('SpringGradient', G, 'G'))
    print(Closure('SpringHessian', H, 'H'))