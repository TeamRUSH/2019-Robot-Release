package frcconf;

import javax.lang.model.element.Element;
import javax.lang.model.element.Name;
import javax.lang.model.type.TypeMirror;

class ConfigurableField {

  Element element;
  Element subElement;

  ConfigurableField(Element element, Element subElement) {
    this.element = element;
    this.subElement = subElement;
  }

  Element getElement() {
    return element;
  }

  Element getSubElement() {
    return subElement;
  }

  TypeMirror getType() {
    return subElement.asType();
  }

  Name getName() {
    return subElement.getSimpleName();
  }

  void check() {
    System.out.println(this.toString());
    System.out.println(String.format("Kind: %s", getType().getKind()));
  }

  boolean isPrimitive() {
    return getType().getKind().isPrimitive() || getType().toString().equals("java.lang.String");
  }

  private String castType() {
    if (getType().toString().equals("java.lang.String")) {
      return "String";
    } else if (getType().getKind().isPrimitive()) {
      switch (getType().getKind()) {
        case INT:
        case DOUBLE:
          return "Double";
        case BOOLEAN:
          return "Boolean";
        default:
      }
    }

    return null;
  }

  private String castFunc() {
    if (getType().toString().equals("java.lang.String")) {
      return "toString";
    } else if (getType().getKind().isPrimitive()) {
      switch (getType().getKind()) {
        case INT:
          return "intValue";
        case DOUBLE:
          return "doubleValue";
        case BOOLEAN:
          return "booleanValue";
        default:
      }
    }

    return null;
  }

  public String formatCast() {
    if (getType().getKind().isPrimitive()) {
      return String.format("((%s) entry.value.getValue()).%s()", castType(), castFunc());
    } else if (getType().toString().equals("java.lang.String")) {
      return "toString()";
    }

    return null;
  }

  @Override
  public String toString() {
    return String.format("{\n\t%s\n\t%s\n}", getType(), getName());
  }
}
