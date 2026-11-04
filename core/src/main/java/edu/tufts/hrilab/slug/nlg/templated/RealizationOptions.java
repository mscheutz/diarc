package edu.tufts.hrilab.slug.nlg.templated;

/** Controls surface variation while preserving semantic content. */
public record RealizationOptions(long seed, boolean varied) {
    public static RealizationOptions canonical() {
	return new RealizationOptions(0L, false); }
    public static RealizationOptions varied(long seed) {
	return new RealizationOptions(seed, true); }
}
