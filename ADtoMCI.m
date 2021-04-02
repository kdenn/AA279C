function MCI = ADtoMCI(AD,GMST)
MCI = rotMCMFtoMCI(GMST)*GDtoMCMF(AD);
end