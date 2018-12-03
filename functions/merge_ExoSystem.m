function out = merge_ExoSystem(Exo1,Exo2)

ExoSys = Exo1;


ExoSys.S = blkdiag(Exo1.S,Exo2.S);
ExoSys.L = blkdiag(Exo1.L , Exo2.L);
ExoSys.X0 = [Exo1.X0; Exo2.X0];
ExoSys.output = [Exo1.output, Exo1.output];
ExoSys.Error = [Exo1.Error , Exo2.Error];
ExoSys.Offset = [Exo1.Offset, Exo2.Offset];

out =ExoSys;
