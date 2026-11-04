package edu.tufts.hrilab.slug.parsing.llm;

import edu.tufts.hrilab.fol.Symbol;

public class AlternateResponse {
  public AlternateResponse (ParserResponse response, Symbol addressee) {
    this.response = response;
    this.addressee = addressee;
  }
  public ParserResponse response;
  public Symbol addressee;
}
