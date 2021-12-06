ar t libalgobsec.a | awk '{
  old=$1;
  new=$1;
  sub(/\.o/, ".c.o", new);
  system("ar x libalgobsec.a "old);
  system("ar dS libalgobsec.a "old);
  system("mv "old" "new);
  system("ar q libalgobsec.a "new);
  system("rm "new)
} END {
  system("ar s libalgobsec.a")
}'
