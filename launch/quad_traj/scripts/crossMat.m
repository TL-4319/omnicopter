function mat = crossMat(qvec)
    mat = [0 -qvec(3) qvec(2);
           qvec(3) 0 -qvec(1);
           -qvec(2) qvec(1) 0];
end