{
  bucket = int(($1 - 1) / SEG)
  cnt[bucket]++
}
END {
  for (i = 0; i < 10; i++) {
    pct = cnt[i] * 100.0 / SEG
    printf "seg%d [%d-%d]: have %d/%d  cover %.1f%%\n", i, i*SEG+1, (i+1)*SEG, cnt[i], SEG, pct
  }
}
