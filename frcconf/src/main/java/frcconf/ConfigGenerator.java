package frcconf;

import com.google.auto.service.AutoService;
import com.google.common.collect.ArrayListMultimap;
import com.google.common.collect.Multimap;
import com.squareup.javapoet.ClassName;
import com.squareup.javapoet.JavaFile;
import com.squareup.javapoet.MethodSpec;
import com.squareup.javapoet.TypeSpec;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PersistentException;
import java.io.IOException;
import java.util.Collection;
import java.util.HashSet;
import java.util.Set;
import javax.annotation.processing.AbstractProcessor;
import javax.annotation.processing.Filer;
import javax.annotation.processing.Messager;
import javax.annotation.processing.ProcessingEnvironment;
import javax.annotation.processing.Processor;
import javax.annotation.processing.RoundEnvironment;
import javax.annotation.processing.SupportedAnnotationTypes;
import javax.annotation.processing.SupportedSourceVersion;
import javax.lang.model.SourceVersion;
import javax.lang.model.element.Element;
import javax.lang.model.element.Modifier;
import javax.lang.model.element.TypeElement;
import javax.lang.model.type.TypeMirror;
import javax.lang.model.util.Elements;

@AutoService(Processor.class)
@SupportedAnnotationTypes({"frcconf.Configurable"})
@SupportedSourceVersion(SourceVersion.RELEASE_11)
public class ConfigGenerator extends AbstractProcessor {
  private static final char CHAR_DOT = '.';

  private Set<Element> configurables;
  private Multimap<TypeMirror, ConfigurableField> typeFields;
  private Messager messager;
  private Filer filer;
  private Elements elements;

  public ConfigGenerator() {
    super();
  }

  @Override
  public synchronized void init(ProcessingEnvironment processingEnv) {
    super.init(processingEnv);

    configurables = new HashSet<>();
    typeFields = ArrayListMultimap.create();
    messager = processingEnv.getMessager();
    filer = processingEnv.getFiler();
    elements = processingEnv.getElementUtils();
  }

  @Override
  public boolean process(Set<? extends TypeElement> annotations, RoundEnvironment roundEnv) {
    if (!roundEnv.processingOver()) {
      for (Element element : roundEnv.getElementsAnnotatedWith(Configurable.class)) {
        configurables.add(element);
        buildClassConfigurableMap(element, typeFields);

        constructConfigClass(element.asType(), typeFields.get(element.asType()), typeFields);
      }
    }

    return true;
  }

  private String shortName(TypeMirror type) {
    String name = type.toString();
    return name.substring(name.lastIndexOf(CHAR_DOT) + 1);
  }

  private String packageName(TypeMirror type) {
    String name = type.toString();
    return name.substring(0, name.lastIndexOf(CHAR_DOT));
  }

  private void constructConfigClass(
      TypeMirror type,
      Collection<ConfigurableField> fields,
      Multimap<TypeMirror, ConfigurableField> typeFields) {
    MethodSpec.Builder defaultBuilder =
        MethodSpec.methodBuilder("putDefault")
            .addModifiers(Modifier.PRIVATE)
            .returns(void.class)
            .addStatement(
                "$T inst = $T.getDefault()",
                NetworkTableInstance.class,
                NetworkTableInstance.class);
    MethodSpec.Builder setupBuilder =
        MethodSpec.methodBuilder("setupNT")
            .addModifiers(Modifier.PRIVATE)
            .returns(void.class)
            .addStatement(
                "$T inst = $T.getDefault()",
                NetworkTableInstance.class,
                NetworkTableInstance.class);

    for (ConfigurableField field : fields) {
      if (field.isPrimitive()) {
        defaultBuilder.addStatement(
            "inst.getEntry($T.format(\"%s/$L\", root).forceSetValue(this.$L)",
            String.class, field.getName(), field.getName());

        setupBuilder.addStatement(
            "inst.addEntryListener(inst.getEntry($T.format(\"%s/$L\", root)), entry ->  this.$L = ($L) entry.value.getValue(), $L)",
            String.class,
            field.getName(),
            field.getName(),
            ClassName.bestGuess(field.getType().toString()),
            EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);
      } else {
        nestedFieldNTStatements(
            field.getName().toString(), defaultBuilder, setupBuilder, field, typeFields);
      }
    }

    MethodSpec defaultNT = defaultBuilder.build();
    MethodSpec setupNT = setupBuilder.build();

    MethodSpec constructor =
        MethodSpec.constructorBuilder()
            .addModifiers(Modifier.PUBLIC)
            .addParameter(String.class, "root")
            .addParameter(String.class, "file")
            .addException(ClassName.get(PersistentException.class))
            .addStatement("this.root = root")
            .addStatement("this.file = file")
            .addStatement("putDefault()")
            .addStatement("$T.getDefault().loadEntries(file, root)", NetworkTableInstance.class)
            .addStatement("setupNT()")
            .addStatement("$T.getDefault().saveEntries(file, root)", NetworkTableInstance.class)
            .build();

    TypeSpec.Builder configBuilder =
        TypeSpec.classBuilder("Configured" + shortName(type))
            .addModifiers(Modifier.PUBLIC)
            .superclass(ClassName.bestGuess(type.toString()))
            .addField(String.class, "root")
            .addField(String.class, "file")
            .addMethod(constructor)
            .addMethod(setupNT)
            .addMethod(defaultNT);

    TypeSpec config = configBuilder.build();

    JavaFile configFile = JavaFile.builder(packageName(type), config).build();

    try {
      configFile.writeTo(filer);
    } catch (IOException ex) {

    }
  }

  private void nestedFieldNTStatements(
      String path,
      MethodSpec.Builder defaultBuilder,
      MethodSpec.Builder setupBuilder,
      ConfigurableField field,
      Multimap<TypeMirror, ConfigurableField> typeFields) {

    for (ConfigurableField subField : typeFields.get(field.getType())) {
      if (subField.isPrimitive()) {
        String property = String.format("%s.%s", path, subField.getName()).replace('/', '.');

        defaultBuilder.addStatement(
            "inst.getEntry($T.format(\"%s/$L/$L\", root)).forceSetValue(this.$L)",
            String.class, path, subField.getName(), property);

        setupBuilder.addStatement(
            "inst.addEntryListener(inst.getEntry($T.format(\"%s/$L/$L\", root)), entry -> this.$L = $L, $L)",
            String.class,
            path,
            subField.getName(),
            property,
            subField.formatCast(),
            EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);
      } else {
        nestedFieldNTStatements(
            String.format("%s/%s", path, subField.getName().toString()),
            defaultBuilder,
            setupBuilder,
            subField,
            typeFields);
      }
    }
  }

  private void buildClassConfigurableMap(
      Element element, Multimap<TypeMirror, ConfigurableField> map) {
    for (Element subElement : element.getEnclosedElements()) {
      switch (subElement.getKind()) {
        case CLASS:
          buildClassConfigurableMap(subElement, map);
          break;
        case FIELD:
          map.put(element.asType(), new ConfigurableField(element, subElement));
          break;
        default:
          break;
      }
    }
  }
}
