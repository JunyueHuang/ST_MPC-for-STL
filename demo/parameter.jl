#system model
A = [1 0.5;
    0.0 1
];

B = [0.5;
     1.0   ];

C = [1.0;
     1.0   ];

X = Hyperrectangle([50.0, 0.0], [50.0, 5.0]);
U = Hyperrectangle([0.0], [2.4]);
W = Hyperrectangle([0.0, 0.0], [0.05, 0.05]);
goal=[Hyperrectangle([13.0, 0.0], [1.0, 5.0]),Hyperrectangle([10.0, 0.0], [5.0, 5.0]),Hyperrectangle([18.0, 0.0], [5.0, 5.0]),Hyperrectangle([13, 0.0], [2, 5.0])]
# goal=[Hyperrectangle([13.0, 0.0], [1.0, 5.0]),Hyperrectangle([10.0, 0.0], [5.0, 5.0]),Hyperrectangle([20.0, 0.0], [5.0, 5.0]),Hyperrectangle([17, 0.0], [3, 5.0])]
goal2l1 = Hyperrectangle([8.5, 0.0], [3.5, 5.0])
goal2r1 = Hyperrectangle([14.5, 0.0], [0.5, 5.0])

Border = [Hyperrectangle([12.0, 0.0], [0.0, 5.0]),Hyperrectangle([14.0, 0.0], [0.0, 5.0])];

begint = [1,13];
endt = [9,21];