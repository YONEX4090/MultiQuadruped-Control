function v = OP_HORZREPSUM()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 102);
  end
  v = vInitialized;
end
